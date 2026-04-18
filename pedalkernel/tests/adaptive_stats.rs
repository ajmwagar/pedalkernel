//! Measures extremum density across pedal circuits for adaptive processing feasibility.
//!
//! Computes what fraction of output samples are at signal extrema (sign change in
//! first derivative). A high skip fraction means sparse/skip processing could be
//! valuable — the signal changes direction infrequently, so between extrema the
//! nonlinear solver could potentially reuse a cached result.
//!
//! Run with:
//!   cargo test --test adaptive_stats -- --nocapture

use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::PedalProcessor;
use std::f64::consts::PI;
use std::path::Path;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

const SAMPLE_RATE: f64 = 48_000.0;
const DURATION_SECS: f64 = 5.0;
const TOTAL_SAMPLES: usize = (SAMPLE_RATE * DURATION_SECS) as usize;
const WARMUP_SAMPLES: usize = 480;

// ---------------------------------------------------------------------------
// Signal generators
// ---------------------------------------------------------------------------

fn sine_signal(freq_hz: f64, amplitude: f64) -> Vec<f64> {
    (0..TOTAL_SAMPLES)
        .map(|i| {
            let t = i as f64 / SAMPLE_RATE;
            amplitude * (2.0 * PI * freq_hz * t).sin()
        })
        .collect()
}

/// Sum of open E major chord partials: E2, A2, D3, G3, B3
fn chord_strum() -> Vec<f64> {
    let freqs = [82.0_f64, 110.0, 147.0, 196.0, 247.0];
    let amp = 0.08;
    (0..TOTAL_SAMPLES)
        .map(|i| {
            let t = i as f64 / SAMPLE_RATE;
            freqs
                .iter()
                .map(|&f| amp * (2.0 * PI * f * t).sin())
                .sum::<f64>()
        })
        .collect()
}

/// 82 Hz sine with exponential decay repeating every 0.25 s — palm mute chug.
fn palm_mute_chug() -> Vec<f64> {
    let freq = 82.0_f64;
    let period = 0.25_f64; // seconds between chug hits
    let decay = 30.0_f64; // fast decay rate
    (0..TOTAL_SAMPLES)
        .map(|i| {
            let t = i as f64 / SAMPLE_RATE;
            let phase_in_period = t % period;
            let env = (-decay * phase_in_period).exp();
            0.4 * env * (2.0 * PI * freq * t).sin()
        })
        .collect()
}

// ---------------------------------------------------------------------------
// AdaptiveStats tracker
// ---------------------------------------------------------------------------

struct AdaptiveStats {
    total_samples: u64,
    extrema_count: u64,
    gap_histogram: [u64; 64], // gaps 1-63, 64+ in last bucket
    max_gap: u32,
    current_gap: u32,
    prev_derivative_sign: bool,
    initialized: bool,
}

impl AdaptiveStats {
    fn new() -> Self {
        Self {
            total_samples: 0,
            extrema_count: 0,
            gap_histogram: [0u64; 64],
            max_gap: 0,
            current_gap: 0,
            prev_derivative_sign: false,
            initialized: false,
        }
    }

    fn record(&mut self, x: f64, x_prev: f64) {
        self.total_samples += 1;
        let d = x - x_prev;
        let sign = d >= 0.0;

        if !self.initialized {
            self.prev_derivative_sign = sign;
            self.initialized = true;
            return;
        }

        if sign != self.prev_derivative_sign {
            // Extremum detected — derivative changed sign
            self.extrema_count += 1;
            let bucket = (self.current_gap as usize).min(63);
            self.gap_histogram[bucket] += 1;
            self.max_gap = self.max_gap.max(self.current_gap);
            self.current_gap = 0;
        } else {
            self.current_gap += 1;
        }
        self.prev_derivative_sign = sign;
    }

    fn skip_fraction(&self) -> f64 {
        if self.total_samples == 0 {
            return 0.0;
        }
        1.0 - (self.extrema_count as f64 / self.total_samples as f64)
    }

    fn median_gap(&self) -> u32 {
        let total: u64 = self.gap_histogram.iter().sum();
        if total == 0 {
            return 0;
        }
        let mut cumulative = 0u64;
        for (i, &count) in self.gap_histogram.iter().enumerate() {
            cumulative += count;
            if cumulative >= total / 2 {
                return i as u32;
            }
        }
        63
    }
}

// ---------------------------------------------------------------------------
// File lookup helper (mirrors audio_analysis.rs pattern)
// ---------------------------------------------------------------------------

fn find_file_recursive(dir: &Path, filename: &str) -> Option<String> {
    if let Ok(entries) = std::fs::read_dir(dir) {
        for entry in entries.flatten() {
            let p = entry.path();
            if p.is_dir() {
                if let Some(found) = find_file_recursive(&p, filename) {
                    return Some(found);
                }
            } else if p.file_name().map_or(false, |n| n == filename) {
                return Some(p.to_string_lossy().to_string());
            }
        }
    }
    None
}

// ---------------------------------------------------------------------------
// Measurement core
// ---------------------------------------------------------------------------

/// Compile a pedal, warm it up, then measure extremum stats over `input`.
/// Returns None if the pedal fails to compile or produces non-finite output.
fn measure_stats(
    pedal_filename: &str,
    controls: &[(&str, f64)],
    input: &[f64],
) -> Option<AdaptiveStats> {
    let examples_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
    let path = find_file_recursive(&examples_dir, pedal_filename)?;
    let src = std::fs::read_to_string(&path).ok()?;

    let pedal = parse_pedal_file(&src).ok()?;
    let mut proc = compile_pedal(&pedal, SAMPLE_RATE).ok()?;

    for &(label, val) in controls {
        proc.set_control(label, val);
    }

    // Warmup: process silence to settle filter state
    for _ in 0..WARMUP_SAMPLES {
        proc.process(0.0);
    }

    // Measure extrema density over the signal
    let mut stats = AdaptiveStats::new();
    let mut prev_out = proc.process(input[0]);
    stats.record(prev_out, 0.0);

    for &sample in &input[1..] {
        let out = proc.process(sample);
        if !out.is_finite() {
            // Non-finite output — pedal diverged, skip this combination
            return None;
        }
        stats.record(out, prev_out);
        prev_out = out;
    }

    Some(stats)
}

// ---------------------------------------------------------------------------
// Table printing
// ---------------------------------------------------------------------------

fn print_table(rows: &[(&str, &str, Option<AdaptiveStats>)]) {
    let col_w_pedal = 18usize;
    let col_w_signal = 18usize;
    let col_w_skip = 11usize;
    let col_w_median = 12usize;
    let col_w_max = 14usize;

    let total_w =
        2 + col_w_pedal + 3 + col_w_signal + 3 + col_w_skip + 3 + col_w_median + 3 + col_w_max + 2;

    // Top border
    println!("╔{}╗", "═".repeat(total_w - 2));

    // Title
    let title = "ADAPTIVE PROCESSING FEASIBILITY";
    let pad = total_w - 2 - title.len();
    let lpad = pad / 2;
    let rpad = pad - lpad;
    println!("║{}{}{}║", " ".repeat(lpad), title, " ".repeat(rpad));

    // Header separator
    println!(
        "╠{}╦{}╦{}╦{}╦{}╣",
        "═".repeat(col_w_pedal + 2),
        "═".repeat(col_w_signal + 2),
        "═".repeat(col_w_skip + 2),
        "═".repeat(col_w_median + 2),
        "═".repeat(col_w_max + 2),
    );

    // Header row
    println!(
        "║ {:<col_w_pedal$} ║ {:<col_w_signal$} ║ {:<col_w_skip$} ║ {:<col_w_median$} ║ {:<col_w_max$} ║",
        "Pedal",
        "Signal",
        "Skip %",
        "Median Gap",
        "Max Gap",
    );

    // Header/data separator
    println!(
        "╠{}╬{}╬{}╬{}╬{}╣",
        "═".repeat(col_w_pedal + 2),
        "═".repeat(col_w_signal + 2),
        "═".repeat(col_w_skip + 2),
        "═".repeat(col_w_median + 2),
        "═".repeat(col_w_max + 2),
    );

    for (pedal, signal, maybe_stats) in rows {
        match maybe_stats {
            Some(stats) => {
                let skip_pct = format!("{:.1}%", stats.skip_fraction() * 100.0);
                let median = stats.median_gap().to_string();
                let max_gap = stats.max_gap.to_string();
                println!(
                    "║ {:<col_w_pedal$} ║ {:<col_w_signal$} ║ {:<col_w_skip$} ║ {:<col_w_median$} ║ {:<col_w_max$} ║",
                    pedal, signal, skip_pct, median, max_gap,
                );
            }
            None => {
                println!(
                    "║ {:<col_w_pedal$} ║ {:<col_w_signal$} ║ {:<col_w_skip$} ║ {:<col_w_median$} ║ {:<col_w_max$} ║",
                    pedal,
                    signal,
                    "FAILED",
                    "-",
                    "-",
                );
            }
        }
    }

    // Bottom border
    println!(
        "╚{}╩{}╩{}╩{}╩{}╝",
        "═".repeat(col_w_pedal + 2),
        "═".repeat(col_w_signal + 2),
        "═".repeat(col_w_skip + 2),
        "═".repeat(col_w_median + 2),
        "═".repeat(col_w_max + 2),
    );
}

// ---------------------------------------------------------------------------
// Test entry point
// ---------------------------------------------------------------------------

#[test]
fn adaptive_processing_feasibility() {
    // Define test signals
    let signals: Vec<(&str, Vec<f64>)> = vec![
        ("Clean strat DI", sine_signal(440.0, 0.1)),
        ("Driven humbucker", sine_signal(440.0, 0.5)),
        ("Chord strum", chord_strum()),
        ("Bass guitar", sine_signal(82.0, 0.3)),
        ("Fingerpicked acoustic", sine_signal(330.0, 0.05)),
        ("Palm mute chug", palm_mute_chug()),
    ];

    // Define pedals: (filename, display_name, controls)
    let pedals: Vec<(&str, &str, Vec<(&str, f64)>)> = vec![
        (
            "tube_screamer.pedal",
            "Tube Screamer",
            vec![("Drive", 0.5), ("Tone", 0.5), ("Level", 0.7)],
        ),
        (
            "proco_rat.pedal",
            "ProCo RAT",
            vec![("Distortion", 0.7), ("Filter", 0.5), ("Volume", 0.5)],
        ),
        (
            "klon_centaur.pedal",
            "Klon Centaur",
            vec![("Gain", 0.5), ("Treble", 0.5), ("Output", 0.7)],
        ),
        (
            "big_muff.pedal",
            "Big Muff Pi",
            vec![("Sustain", 0.7), ("Tone", 0.5), ("Volume", 0.5)],
        ),
        (
            "blues_driver.pedal",
            "Blues Driver",
            vec![("Gain", 0.5), ("Tone", 0.5), ("Level", 0.7)],
        ),
        (
            "dyna_comp.pedal",
            "Dyna Comp",
            vec![("Sensitivity", 0.5), ("Output", 0.7)],
        ),
    ];

    // Collect all results
    let mut rows: Vec<(&str, &str, Option<AdaptiveStats>)> = Vec::new();
    let mut total_skip_fracs: Vec<f64> = Vec::new();

    for (filename, pedal_name, controls) in &pedals {
        for (signal_name, signal) in &signals {
            let stats = measure_stats(filename, controls, signal);
            if let Some(ref s) = stats {
                total_skip_fracs.push(s.skip_fraction());
            } else {
                eprintln!(
                    "  [warn] {pedal_name} / {signal_name}: skipped (compile/process failure)"
                );
            }
            rows.push((pedal_name, signal_name, stats));
        }
    }

    // Print the results table
    println!();
    print_table(&rows);
    println!();

    // Compute overall verdict
    let avg_skip = if total_skip_fracs.is_empty() {
        0.0
    } else {
        total_skip_fracs.iter().sum::<f64>() / total_skip_fracs.len() as f64
    };

    let pct = avg_skip * 100.0;
    let verdict = if pct > 50.0 {
        "Adaptive processing is WORTH PURSUING"
    } else if pct >= 30.0 {
        "Marginal -- signal-dependent, needs per-pedal decision"
    } else {
        "NOT WORTH IT -- extrema are too frequent"
    };

    println!("VERDICT: {pct:.1}% average skip fraction across all combinations.");
    println!("  > 50%: Adaptive processing is WORTH PURSUING");
    println!("  30-50%: Marginal -- signal-dependent, needs per-pedal decision");
    println!("  < 30%: NOT WORTH IT -- extrema are too frequent");
    println!();
    println!("  => {verdict}");
    println!();
}
