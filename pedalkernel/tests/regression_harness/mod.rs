#![allow(dead_code)]

use pedalkernel::compiler::{compile_pedal_with_options, CompileOptions, CompiledPedal};
use pedalkernel::dsl::{parse_pedal_file, PedalDef};
use pedalkernel::oversampling::OversamplingFactor;
use pedalkernel::PedalProcessor;
use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::PI;
use std::fs;
use std::panic::{self, AssertUnwindSafe};
use std::path::PathBuf;

pub const SAMPLE_RATE: f64 = 48_000.0;
const WARMUP: usize = 12_000;
const MEASURE: usize = 12_000;
const EPS: f64 = 1.0e-12;

#[derive(Clone, Copy, Debug)]
pub enum CompileVariant {
    Default,
    DisableIir,
    SkipBlockwise,
    SkipKTables,
    OversamplingX1,
}

impl CompileVariant {
    pub fn name(self) -> &'static str {
        match self {
            CompileVariant::Default => "default",
            CompileVariant::DisableIir => "disable_iir",
            CompileVariant::SkipBlockwise => "skip_blockwise",
            CompileVariant::SkipKTables => "skip_k_tables",
            CompileVariant::OversamplingX1 => "oversampling_x1",
        }
    }

    fn options(self) -> CompileOptions {
        match self {
            CompileVariant::Default => CompileOptions::default(),
            CompileVariant::DisableIir => CompileOptions {
                disable_iir: true,
                skip_k_tables: true,
                ..Default::default()
            },
            CompileVariant::SkipBlockwise => CompileOptions {
                skip_blockwise: true,
                skip_k_tables: true,
                ..Default::default()
            },
            CompileVariant::SkipKTables => CompileOptions {
                skip_k_tables: true,
                ..Default::default()
            },
            CompileVariant::OversamplingX1 => CompileOptions {
                oversampling: OversamplingFactor::X1,
                skip_k_tables: true,
                ..Default::default()
            },
        }
    }
}

pub const STANDARD_VARIANTS: &[CompileVariant] = &[
    CompileVariant::Default,
    CompileVariant::DisableIir,
    CompileVariant::SkipBlockwise,
    CompileVariant::SkipKTables,
    CompileVariant::OversamplingX1,
];

#[derive(Clone, Copy, Debug)]
pub enum ControlBehavior {
    DriveHarmonics,
    DriveGetsDirtier,
    DecayTail,
    ToneTilt,
    OutputLevel,
    MovesOutput,
}

#[derive(Clone, Copy, Debug)]
pub struct ControlCheck {
    pub label: &'static str,
    pub behavior: ControlBehavior,
    pub min_delta_db: f64,
    pub min_waveform_diff: f64,
}

impl ControlCheck {
    pub const fn drive(label: &'static str, min_harmonic_delta_db: f64) -> Self {
        Self {
            label,
            behavior: ControlBehavior::DriveHarmonics,
            min_delta_db: min_harmonic_delta_db,
            min_waveform_diff: 0.01,
        }
    }

    pub const fn dirtier_drive(label: &'static str, min_harmonic_energy_lift_db: f64) -> Self {
        Self {
            label,
            behavior: ControlBehavior::DriveGetsDirtier,
            min_delta_db: min_harmonic_energy_lift_db,
            min_waveform_diff: 0.01,
        }
    }

    pub const fn decay(label: &'static str, min_tail_lift_db: f64) -> Self {
        Self {
            label,
            behavior: ControlBehavior::DecayTail,
            min_delta_db: min_tail_lift_db,
            min_waveform_diff: 0.01,
        }
    }

    pub const fn tone(label: &'static str, min_tilt_delta_db: f64) -> Self {
        Self {
            label,
            behavior: ControlBehavior::ToneTilt,
            min_delta_db: min_tilt_delta_db,
            min_waveform_diff: 0.005,
        }
    }

    pub const fn output(label: &'static str, min_level_delta_db: f64) -> Self {
        Self {
            label,
            behavior: ControlBehavior::OutputLevel,
            min_delta_db: min_level_delta_db,
            min_waveform_diff: 0.01,
        }
    }

    pub const fn moves(label: &'static str, min_waveform_diff: f64) -> Self {
        Self {
            label,
            behavior: ControlBehavior::MovesOutput,
            min_delta_db: 0.0,
            min_waveform_diff,
        }
    }
}

pub struct CircuitRegressionSpec {
    pub name: &'static str,
    pub source: &'static str,
    pub checks: &'static [ControlCheck],
}

#[derive(Default)]
pub struct RegressionReport {
    lines: Vec<String>,
    failures: Vec<String>,
}

impl RegressionReport {
    pub fn fail(&mut self, msg: impl Into<String>) {
        self.failures.push(msg.into());
    }

    pub fn line(&mut self, line: impl Into<String>) {
        self.lines.push(line.into());
    }

    pub fn finish(self, suite: &str) {
        write_report(suite, &self.lines, &self.failures);
        assert!(
            self.failures.is_empty(),
            "{suite} regression failures:\n{}",
            self.failures.join("\n")
        );
    }
}

pub fn run_circuit_matrix(specs: &[CircuitRegressionSpec], variants: &[CompileVariant]) {
    run_circuit_matrix_named("audio_matrix", specs, variants);
}

pub fn run_circuit_matrix_named(
    suite: &str,
    specs: &[CircuitRegressionSpec],
    variants: &[CompileVariant],
) {
    let mut report = RegressionReport::default();
    report.line("# Pedalkernel Audio Regression Matrix");
    report.line("");

    for spec in specs {
        if let Err(payload) = panic::catch_unwind(AssertUnwindSafe(|| {
            run_one_circuit(spec, variants, &mut report);
        })) {
            report.fail(format!(
                "{}: panicked while running regression matrix: {}",
                spec.name,
                panic_message(payload)
            ));
        }
    }

    report.finish(suite);
}

pub fn assert_circuit_specs_parse_and_controls_mapped(specs: &[CircuitRegressionSpec]) {
    for spec in specs {
        let pedal = parse_pedal_file(spec.source)
            .unwrap_or_else(|e| panic!("{}: parse failed: {e}", spec.name));
        let labels: BTreeSet<&str> = pedal.controls.iter().map(|c| c.label.as_str()).collect();
        let checked: BTreeSet<&str> = spec.checks.iter().map(|check| check.label).collect();

        for check in spec.checks {
            assert!(
                labels.contains(check.label),
                "{}: control {:?} not found in parsed controls {:?}",
                spec.name,
                check.label,
                labels
            );
        }

        for label in labels.difference(&checked) {
            panic!(
                "{}: parsed .pedal control {:?} has no regression behavior mapping",
                spec.name, label
            );
        }
    }
}

fn panic_message(payload: Box<dyn std::any::Any + Send>) -> String {
    if let Some(msg) = payload.downcast_ref::<&str>() {
        (*msg).to_string()
    } else if let Some(msg) = payload.downcast_ref::<String>() {
        msg.clone()
    } else {
        "non-string panic payload".to_string()
    }
}

fn run_one_circuit(
    spec: &CircuitRegressionSpec,
    variants: &[CompileVariant],
    report: &mut RegressionReport,
) {
    let pedal = match parse_pedal_file(spec.source) {
        Ok(p) => p,
        Err(e) => {
            report.fail(format!("{}: parse failed: {e}", spec.name));
            return;
        }
    };
    validate_check_labels(spec, &pedal, report);
    let defaults = control_defaults(&pedal);

    report.line(format!("## {}", spec.name));
    report.line("");

    for &variant in variants {
        let variant_name = variant.name();
        let mut compiled = match compile_pedal_with_options(&pedal, SAMPLE_RATE, variant.options())
        {
            Ok(c) => c,
            Err(e) => {
                report.fail(format!(
                    "{}[{variant_name}]: compile failed: {e}",
                    spec.name
                ));
                continue;
            }
        };
        let smoke = render_compiled(&mut compiled, 1_000.0, 0.1);
        let health = health_metrics(&smoke);
        report.line(format!(
            "- `{variant_name}` health: rms={:.6}, peak={:.6}, dc={:.3e}",
            health.rms, health.peak, health.dc
        ));
        if !health.finite || health.rms < 1e-7 || health.peak > 100.0 {
            report.fail(format!(
                "{}[{variant_name}]: unhealthy output: finite={}, rms={:.6}, peak={:.6}",
                spec.name, health.finite, health.rms, health.peak
            ));
        }
    }

    for &check in spec.checks {
        run_control_check(spec, &pedal, &defaults, check, report);
    }

    run_frequency_sweep(spec, &pedal, &defaults, report);
    report.line("");
}

fn validate_check_labels(
    spec: &CircuitRegressionSpec,
    pedal: &PedalDef,
    report: &mut RegressionReport,
) {
    let labels: BTreeSet<&str> = pedal.controls.iter().map(|c| c.label.as_str()).collect();
    let checked: BTreeSet<&str> = spec.checks.iter().map(|check| check.label).collect();
    for check in spec.checks {
        if !labels.contains(check.label) {
            report.fail(format!(
                "{}: control {:?} not found in parsed controls {:?}",
                spec.name, check.label, labels
            ));
        }
    }
    for label in labels.difference(&checked) {
        report.fail(format!(
            "{}: parsed .pedal control {:?} has no regression behavior mapping",
            spec.name, label
        ));
    }
}

fn control_defaults(pedal: &PedalDef) -> BTreeMap<String, f64> {
    let mut defaults = BTreeMap::new();
    for control in &pedal.controls {
        defaults
            .entry(control.label.clone())
            .or_insert(control.default.clamp(0.0, 1.0));
    }
    defaults
}

fn control_range(pedal: &PedalDef, label: &str) -> Option<(f64, f64)> {
    pedal
        .controls
        .iter()
        .find(|control| control.label == label)
        .map(|control| control.range)
}

fn map_control_range(range: (f64, f64), value: f64) -> f64 {
    let (r0, r1) = range;
    (r0 + value.clamp(0.0, 1.0) * (r1 - r0)).clamp(0.0, 1.0)
}

fn run_control_check(
    spec: &CircuitRegressionSpec,
    pedal: &PedalDef,
    defaults: &BTreeMap<String, f64>,
    check: ControlCheck,
    report: &mut RegressionReport,
) {
    let range = control_range(pedal, check.label).unwrap_or((0.0, 1.0));
    let low_controls = controls_with(defaults, check.label, 0.0);
    let high_controls = controls_with(defaults, check.label, 1.0);
    let neutral_controls = defaults
        .iter()
        .map(|(k, v)| (k.as_str(), *v))
        .collect::<Vec<_>>();

    match check.behavior {
        ControlBehavior::DriveHarmonics => {
            let low = render(pedal, CompileVariant::Default, &low_controls, 1_000.0, 0.18);
            let high = render(
                pedal,
                CompileVariant::Default,
                &high_controls,
                1_000.0,
                0.18,
            );
            let low_h = harmonics(&low, 1_000.0);
            let high_h = harmonics(&high, 1_000.0);
            let delta = high_h.thd_db - low_h.thd_db;
            let diff = normalized_waveform_diff(&low, &high);
            report.line(format!(
                "- `{}` drive: thd {:.2}dB -> {:.2}dB (delta {:.2}dB), diff={:.4}",
                check.label, low_h.thd_db, high_h.thd_db, delta, diff
            ));
            if delta.abs() < check.min_delta_db && diff < check.min_waveform_diff {
                report.fail(format!(
                    "{} default {} drive did not move harmonics enough: delta={delta:.2}dB diff={diff:.4}",
                    spec.name, check.label
                ));
            }
        }
        ControlBehavior::DriveGetsDirtier => {
            let ext_a = 0.0;
            let ext_b = 1.0;
            let raw_a = map_control_range(range, ext_a);
            let raw_b = map_control_range(range, ext_b);
            let a_controls = controls_with(defaults, check.label, ext_a);
            let b_controls = controls_with(defaults, check.label, ext_b);
            let a = render(pedal, CompileVariant::Default, &a_controls, 1_000.0, 0.18);
            let b = render(pedal, CompileVariant::Default, &b_controls, 1_000.0, 0.18);
            let (cleaner, dirtier, clean_raw, dirty_raw, clean_ext, dirty_ext) = if raw_a <= raw_b {
                (&a, &b, raw_a, raw_b, ext_a, ext_b)
            } else {
                (&b, &a, raw_b, raw_a, ext_b, ext_a)
            };
            let cleaner_h = harmonics(cleaner, 1_000.0);
            let dirtier_h = harmonics(dirtier, 1_000.0);
            let cleaner_energy_db = db(cleaner_h.harmonic_rms);
            let dirtier_energy_db = db(dirtier_h.harmonic_rms);
            let energy_lift = dirtier_energy_db - cleaner_energy_db;
            let cleaner_crest = crest_factor(cleaner);
            let dirtier_crest = crest_factor(dirtier);
            let crest_drop = cleaner_crest - dirtier_crest;
            let diff = normalized_waveform_diff(cleaner, dirtier);
            report.line(format!(
                "- `{}` dirtier drive: ext {:.1}->{:.1} maps raw {:.1}->{:.1}; harmonic_energy {:.2}dB -> {:.2}dB (lift {:.2}dB), crest {:.3} -> {:.3} (drop {:.3}), diff={:.4}",
                check.label,
                clean_ext,
                dirty_ext,
                clean_raw,
                dirty_raw,
                cleaner_energy_db,
                dirtier_energy_db,
                energy_lift,
                cleaner_crest,
                dirtier_crest,
                crest_drop,
                diff
            ));
            if energy_lift < check.min_delta_db {
                report.fail(format!(
                    "{} default {} raw-high drive did not raise absolute harmonic energy enough: lift={energy_lift:.2}dB",
                    spec.name, check.label
                ));
            }
            if crest_drop < 0.05 && diff < check.min_waveform_diff {
                report.fail(format!(
                    "{} default {} raw-high drive did not show clipping-shaped movement: crest_drop={crest_drop:.3} diff={diff:.4}",
                    spec.name, check.label
                ));
            }
        }
        ControlBehavior::DecayTail => {
            let ext_a = 0.0;
            let ext_b = 1.0;
            let raw_a = map_control_range(range, ext_a);
            let raw_b = map_control_range(range, ext_b);
            let a_controls = controls_with(defaults, check.label, ext_a);
            let b_controls = controls_with(defaults, check.label, ext_b);
            let a = render_impulse(pedal, CompileVariant::Default, &a_controls, 1.0);
            let b = render_impulse(pedal, CompileVariant::Default, &b_controls, 1.0);
            let (shorter, longer, short_raw, long_raw, short_ext, long_ext) = if raw_a <= raw_b {
                (&a, &b, raw_a, raw_b, ext_a, ext_b)
            } else {
                (&b, &a, raw_b, raw_a, ext_b, ext_a)
            };
            let shorter_tail = tail_rms(shorter);
            let longer_tail = tail_rms(longer);
            let tail_lift = db(longer_tail) - db(shorter_tail);
            let diff = normalized_waveform_diff(shorter, longer);
            report.line(format!(
                "- `{}` decay: ext {:.1}->{:.1} maps raw {:.1}->{:.1}; tail {:.2}dB -> {:.2}dB (lift {:.2}dB), diff={:.4}",
                check.label,
                short_ext,
                long_ext,
                short_raw,
                long_raw,
                db(shorter_tail),
                db(longer_tail),
                tail_lift,
                diff
            ));
            if tail_lift < check.min_delta_db && diff < check.min_waveform_diff {
                report.fail(format!(
                    "{} default {} raw-high decay did not increase tail enough: lift={tail_lift:.2}dB diff={diff:.4}",
                    spec.name, check.label
                ));
            }
        }
        ControlBehavior::ToneTilt => {
            let low_tilt = spectral_tilt_db(pedal, CompileVariant::Default, &low_controls, 0.08);
            let high_tilt = spectral_tilt_db(pedal, CompileVariant::Default, &high_controls, 0.08);
            let delta = high_tilt - low_tilt;
            let low = render(pedal, CompileVariant::Default, &low_controls, 1_000.0, 0.08);
            let high = render(
                pedal,
                CompileVariant::Default,
                &high_controls,
                1_000.0,
                0.08,
            );
            let diff = normalized_waveform_diff(&low, &high);
            report.line(format!(
                "- `{}` tone: tilt {:.2}dB -> {:.2}dB (delta {:.2}dB), diff={:.4}",
                check.label, low_tilt, high_tilt, delta, diff
            ));
            if delta.abs() < check.min_delta_db && diff < check.min_waveform_diff {
                report.fail(format!(
                    "{} default {} tone/filter did not move spectrum enough: delta={delta:.2}dB diff={diff:.4}",
                    spec.name, check.label
                ));
            }
        }
        ControlBehavior::OutputLevel => {
            let low = render(pedal, CompileVariant::Default, &low_controls, 1_000.0, 0.08);
            let high = render(
                pedal,
                CompileVariant::Default,
                &high_controls,
                1_000.0,
                0.08,
            );
            let low_db = db(rms(&low));
            let high_db = db(rms(&high));
            let delta = (high_db - low_db).abs();
            report.line(format!(
                "- `{}` output: rms {:.2}dB -> {:.2}dB (delta {:.2}dB)",
                check.label, low_db, high_db, delta
            ));
            if delta < check.min_delta_db {
                report.fail(format!(
                    "{} default {} output level did not move enough: delta={delta:.2}dB",
                    spec.name, check.label
                ));
            }
        }
        ControlBehavior::MovesOutput => {
            let neutral = render(
                pedal,
                CompileVariant::Default,
                &neutral_controls,
                1_000.0,
                0.08,
            );
            let high = render(
                pedal,
                CompileVariant::Default,
                &high_controls,
                1_000.0,
                0.08,
            );
            let diff = normalized_waveform_diff(&neutral, &high);
            report.line(format!("- `{}` movement: diff={diff:.4}", check.label));
            if diff < check.min_waveform_diff {
                report.fail(format!(
                    "{} default {} did not change output enough: diff={diff:.4}",
                    spec.name, check.label
                ));
            }
        }
    }

    // Differential probe: if default is weak but disable_iir moves, blame IIR.
    let default_low = render(pedal, CompileVariant::Default, &low_controls, 1_000.0, 0.08);
    let default_high = render(
        pedal,
        CompileVariant::Default,
        &high_controls,
        1_000.0,
        0.08,
    );
    let noiir_low = try_render(
        pedal,
        CompileVariant::DisableIir,
        &low_controls,
        1_000.0,
        0.08,
    );
    let noiir_high = try_render(
        pedal,
        CompileVariant::DisableIir,
        &high_controls,
        1_000.0,
        0.08,
    );
    let default_diff = normalized_waveform_diff(&default_low, &default_high);
    match (noiir_low, noiir_high) {
        (Ok(noiir_low), Ok(noiir_high)) => {
            let noiir_diff = normalized_waveform_diff(&noiir_low, &noiir_high);
            if noiir_diff > default_diff * 3.0 && noiir_diff > 0.01 {
                report.line(format!(
                    "  - differential: `disable_iir` moves `{}` more strongly ({:.4} vs {:.4})",
                    check.label, noiir_diff, default_diff
                ));
            }
        }
        (Err(err), _) | (_, Err(err)) => {
            report.line(format!(
                "  - differential: `disable_iir` skipped for `{}` ({err})",
                check.label
            ));
        }
    }
}

fn run_frequency_sweep(
    spec: &CircuitRegressionSpec,
    pedal: &PedalDef,
    defaults: &BTreeMap<String, f64>,
    report: &mut RegressionReport,
) {
    let controls = defaults
        .iter()
        .map(|(k, v)| (k.as_str(), *v))
        .collect::<Vec<_>>();
    let freqs = [
        80.0, 160.0, 320.0, 640.0, 1_250.0, 2_500.0, 5_000.0, 10_000.0,
    ];
    let mut gains = Vec::with_capacity(freqs.len());
    for freq in freqs {
        gains.push(gain_db(
            pedal,
            CompileVariant::Default,
            &controls,
            freq,
            0.05,
        ));
    }
    let min = gains.iter().copied().fold(f64::INFINITY, f64::min);
    let max = gains.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    report.line(format!(
        "- frequency sweep: min={min:.2}dB max={max:.2}dB span={:.2}dB",
        max - min
    ));
    if !min.is_finite() || !max.is_finite() || max > 80.0 {
        report.fail(format!(
            "{} frequency sweep unstable: min={min:.2} max={max:.2}",
            spec.name
        ));
    }
}

fn controls_with<'a>(
    defaults: &'a BTreeMap<String, f64>,
    label: &'a str,
    value: f64,
) -> Vec<(&'a str, f64)> {
    let mut controls = defaults
        .iter()
        .map(|(k, v)| (k.as_str(), *v))
        .collect::<Vec<_>>();
    if let Some((_, v)) = controls.iter_mut().find(|(name, _)| *name == label) {
        *v = value;
    } else {
        controls.push((label, value));
    }
    controls
}

fn compile_processor(
    pedal: &PedalDef,
    variant: CompileVariant,
    controls: &[(&str, f64)],
) -> CompiledPedal {
    let mut proc =
        compile_pedal_with_options(pedal, SAMPLE_RATE, variant.options()).expect("compile");
    for &(label, value) in controls {
        proc.set_control(label, value);
    }
    proc
}

fn render(
    pedal: &PedalDef,
    variant: CompileVariant,
    controls: &[(&str, f64)],
    freq: f64,
    amplitude: f64,
) -> Vec<f64> {
    let mut proc = compile_processor(pedal, variant, controls);
    render_compiled(&mut proc, freq, amplitude)
}

fn try_render(
    pedal: &PedalDef,
    variant: CompileVariant,
    controls: &[(&str, f64)],
    freq: f64,
    amplitude: f64,
) -> Result<Vec<f64>, String> {
    panic::catch_unwind(AssertUnwindSafe(|| {
        render(pedal, variant, controls, freq, amplitude)
    }))
    .map_err(panic_message)
}

fn render_compiled(proc: &mut CompiledPedal, freq: f64, amplitude: f64) -> Vec<f64> {
    for i in 0..WARMUP {
        let input = amplitude * (2.0 * PI * freq * i as f64 / SAMPLE_RATE).sin();
        proc.process(input);
    }

    (0..MEASURE)
        .map(|i| {
            let n = WARMUP + i;
            let input = amplitude * (2.0 * PI * freq * n as f64 / SAMPLE_RATE).sin();
            proc.process(input)
        })
        .collect()
}

fn render_impulse(
    pedal: &PedalDef,
    variant: CompileVariant,
    controls: &[(&str, f64)],
    amplitude: f64,
) -> Vec<f64> {
    let mut proc = compile_processor(pedal, variant, controls);
    for _ in 0..WARMUP {
        proc.process(0.0);
    }
    let mut out = Vec::with_capacity(MEASURE);
    out.push(proc.process(amplitude));
    for _ in 1..MEASURE {
        out.push(proc.process(0.0));
    }
    out
}

fn gain_db(
    pedal: &PedalDef,
    variant: CompileVariant,
    controls: &[(&str, f64)],
    freq: f64,
    amplitude: f64,
) -> f64 {
    let out = render(pedal, variant, controls, freq, amplitude);
    let amp = projected_amplitude(&out, freq);
    db(amp / amplitude.max(EPS))
}

fn spectral_tilt_db(
    pedal: &PedalDef,
    variant: CompileVariant,
    controls: &[(&str, f64)],
    amplitude: f64,
) -> f64 {
    gain_db(pedal, variant, controls, 5_000.0, amplitude)
        - gain_db(pedal, variant, controls, 500.0, amplitude)
}

#[derive(Debug)]
struct HealthMetrics {
    finite: bool,
    rms: f64,
    peak: f64,
    dc: f64,
}

fn health_metrics(samples: &[f64]) -> HealthMetrics {
    HealthMetrics {
        finite: samples.iter().all(|s| s.is_finite()),
        rms: rms(samples),
        peak: samples.iter().map(|s| s.abs()).fold(0.0, f64::max),
        dc: if samples.is_empty() {
            0.0
        } else {
            samples.iter().sum::<f64>() / samples.len() as f64
        },
    }
}

#[derive(Debug)]
struct Harmonics {
    fundamental: f64,
    harmonics: Vec<f64>,
    harmonic_rms: f64,
    thd_db: f64,
    even_odd_db: f64,
}

fn harmonics(samples: &[f64], fundamental_hz: f64) -> Harmonics {
    let fundamental = projected_amplitude(samples, fundamental_hz);
    let mut hs = Vec::new();
    for n in 2..=10 {
        hs.push(projected_amplitude(samples, fundamental_hz * n as f64));
    }

    let thd = hs.iter().map(|h| h * h).sum::<f64>().sqrt() / fundamental.max(EPS);
    let even = hs
        .iter()
        .enumerate()
        .filter(|(i, _)| (i + 2) % 2 == 0)
        .map(|(_, h)| h * h)
        .sum::<f64>()
        .sqrt();
    let odd = hs
        .iter()
        .enumerate()
        .filter(|(i, _)| (i + 2) % 2 == 1)
        .map(|(_, h)| h * h)
        .sum::<f64>()
        .sqrt();

    Harmonics {
        fundamental,
        harmonics: hs,
        harmonic_rms: thd * fundamental,
        thd_db: db(thd),
        even_odd_db: db(even / odd.max(EPS)),
    }
}

fn projected_amplitude(samples: &[f64], freq: f64) -> f64 {
    let mut sin_sum = 0.0;
    let mut cos_sum = 0.0;
    for (i, &s) in samples.iter().enumerate() {
        let phase = 2.0 * PI * freq * i as f64 / SAMPLE_RATE;
        sin_sum += s * phase.sin();
        cos_sum += s * phase.cos();
    }
    2.0 * (sin_sum.hypot(cos_sum)) / samples.len() as f64
}

fn normalized_waveform_diff(a: &[f64], b: &[f64]) -> f64 {
    let n = a.len().min(b.len());
    if n == 0 {
        return 0.0;
    }
    let diff = (0..n)
        .map(|i| {
            let d = a[i] - b[i];
            d * d
        })
        .sum::<f64>()
        / n as f64;
    diff.sqrt() / rms(a).max(rms(b)).max(EPS)
}

fn rms(samples: &[f64]) -> f64 {
    if samples.is_empty() {
        return 0.0;
    }
    (samples.iter().map(|s| s * s).sum::<f64>() / samples.len() as f64).sqrt()
}

fn tail_rms(samples: &[f64]) -> f64 {
    if samples.len() < 4 {
        return rms(samples);
    }
    let start = samples.len() * 3 / 4;
    rms(&samples[start..])
}

fn crest_factor(samples: &[f64]) -> f64 {
    let peak = samples.iter().map(|s| s.abs()).fold(0.0, f64::max);
    peak / rms(samples).max(EPS)
}

fn db(value: f64) -> f64 {
    20.0 * value.abs().max(EPS).log10()
}

fn write_report(suite: &str, lines: &[String], failures: &[String]) {
    let mut path = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    path.push("target");
    path.push("regression");
    if fs::create_dir_all(&path).is_err() {
        return;
    }
    path.push(format!("{suite}.md"));

    let mut out = lines.join("\n");
    out.push_str("\n\n## Failures\n\n");
    if failures.is_empty() {
        out.push_str("None\n");
    } else {
        for failure in failures {
            out.push_str("- ");
            out.push_str(failure);
            out.push('\n');
        }
    }
    let _ = fs::write(path, out);
}
