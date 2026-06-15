use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};
use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::elements::*;
use pedalkernel::tree::*;
use pedalkernel::PedalProcessor;
use std::time::{Duration, Instant};

const SAMPLE_RATE: f64 = 48_000.0;
const BLOCK_SIZES: &[usize] = &[64, 128, 256, 512];

// ═══════════════════════════════════════════════════════════════════
// FLOPS estimation constants (approximate operation counts)
// ═══════════════════════════════════════════════════════════════════

/// Estimated FLOPs per Newton-Raphson iteration for diode solving.
/// Breakdown: 2 exp() (~20 each), 6 mul, 4 add, 1 div, 1 clamp ≈ 55 FLOPs
const FLOPS_PER_NEWTON_ITER: u64 = 55;

/// Average Newton iterations for diode convergence (measured empirically).
const AVG_NEWTON_ITERS: u64 = 4;

/// FLOPs for series adaptor scatter (scatter_up + scatter_down).
/// scatter_up: 2 add, 1 neg = 3; scatter_down: 3 add, 2 mul, 1 sub = 6
const FLOPS_SERIES_ADAPTOR: u64 = 9;

/// FLOPs for parallel adaptor scatter.
/// scatter_up: 2 sub, 1 mul, 1 add = 4; scatter_down: 2 sub, 2 mul, 2 add = 6
const FLOPS_PARALLEL_ADAPTOR: u64 = 10;

/// FLOPs for capacitor state update (1 mul, 1 sub).
const FLOPS_CAPACITOR: u64 = 2;

/// FLOPs for resistor (trivial).
const FLOPS_RESISTOR: u64 = 1;

/// FLOPs for voltage source set + reflected.
const FLOPS_VOLTAGE_SOURCE: u64 = 2;

/// FLOPs for final output computation (add + div).
const FLOPS_OUTPUT: u64 = 2;

fn test_block(size: usize) -> Vec<f64> {
    (0..size)
        .map(|i| 0.5 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / SAMPLE_RATE).sin())
        .collect()
}

/// Estimate FLOPs per sample for a simple WDF clipper circuit.
/// Tree: VoltageSource -> Series -> (Parallel -> (Resistor, Capacitor)) -> DiodePair
fn estimate_clipper_flops() -> u64 {
    FLOPS_VOLTAGE_SOURCE
        + FLOPS_RESISTOR
        + FLOPS_CAPACITOR
        + FLOPS_PARALLEL_ADAPTOR
        + FLOPS_SERIES_ADAPTOR
        + (FLOPS_PER_NEWTON_ITER * AVG_NEWTON_ITERS) // Diode pair Newton solve
        + FLOPS_OUTPUT
}

/// Format FLOPS as human-readable string (MFLOPS, GFLOPS).
fn format_flops(flops: f64) -> String {
    if flops >= 1e9 {
        format!("{:.2} GFLOPS", flops / 1e9)
    } else if flops >= 1e6 {
        format!("{:.2} MFLOPS", flops / 1e6)
    } else if flops >= 1e3 {
        format!("{:.2} KFLOPS", flops / 1e3)
    } else {
        format!("{:.0} FLOPS", flops)
    }
}

fn compile_example(name: &str) -> Box<dyn PedalProcessor> {
    let path = find_example(name);
    let src = std::fs::read_to_string(&path).unwrap();
    let pedal = parse_pedal_file(&src).unwrap();
    Box::new(compile_pedal(&pedal, SAMPLE_RATE).unwrap())
}

fn compile_pedal_file(path: &str) -> Box<dyn PedalProcessor> {
    let src = std::fs::read_to_string(path).unwrap();
    let pedal = parse_pedal_file(&src).unwrap();
    Box::new(compile_pedal(&pedal, SAMPLE_RATE).unwrap())
}

fn find_example(filename: &str) -> String {
    fn walk(dir: &str, target: &str) -> Option<String> {
        for entry in std::fs::read_dir(dir).ok()?.flatten() {
            let path = entry.path();
            if path.is_dir() {
                if let Some(found) = walk(path.to_str()?, target) {
                    return Some(found);
                }
            } else if path.file_name().and_then(|n| n.to_str()) == Some(target) {
                return Some(path.to_string_lossy().to_string());
            }
        }
        None
    }
    walk("examples", filename).unwrap_or_else(|| panic!("example file not found: {filename}"))
}

// ═══════════════════════════════════════════════════════════════════
// Group 1: Existing benchmarks (preserved)
// ═══════════════════════════════════════════════════════════════════

fn bench_wdf_clipper(c: &mut Criterion) {
    let mut clipper = WdfClipper::new(4700.0, 220e-9, DiodeModel::silicon(), SAMPLE_RATE);

    c.bench_function("wdf_clipper_sample", |b| {
        let mut phase = 0.0_f64;
        b.iter(|| {
            phase += 440.0 / SAMPLE_RATE;
            let input = 0.5 * (2.0 * std::f64::consts::PI * phase).sin();
            black_box(clipper.process(black_box(input)))
        })
    });
}

fn bench_wdf_clipper_block(c: &mut Criterion) {
    let mut clipper = WdfClipper::new(4700.0, 220e-9, DiodeModel::silicon(), SAMPLE_RATE);
    let block = test_block(64);

    c.bench_function("wdf_clipper_64_samples", |b| {
        b.iter(|| {
            for &s in &block {
                black_box(clipper.process(black_box(s)));
            }
        })
    });
}

fn bench_diode_pair_newton(c: &mut Criterion) {
    let mut dp = DiodePairRoot::new(DiodeModel::silicon());

    c.bench_function("diode_pair_newton", |b| {
        b.iter(|| black_box(dp.process(black_box(0.5), black_box(4700.0))))
    });
}

// ═══════════════════════════════════════════════════════════════════
// Group 2: All compiled pedal processors — per-sample
// ═══════════════════════════════════════════════════════════════════

fn bench_compiled_pedals_sample(c: &mut Criterion) {
    let mut group = c.benchmark_group("compiled_pedal_sample");

    let pedals = [
        "tube_screamer.pedal",
        "big_muff.pedal",
        "fuzz_face.pedal",
        "blues_driver.pedal",
        "dyna_comp.pedal",
        "klon_centaur.pedal",
        "proco_rat.pedal",
    ];

    for name in pedals {
        let mut proc = compile_example(name);
        let label = name.trim_end_matches(".pedal");
        group.bench_function(label, |b| {
            let mut phase = 0.0_f64;
            b.iter(|| {
                phase += 440.0 / SAMPLE_RATE;
                let input = 0.5 * (2.0 * std::f64::consts::PI * phase).sin();
                black_box(proc.process(black_box(input)))
            })
        });
    }

    group.finish();
}

// ═══════════════════════════════════════════════════════════════════
// Group 3: Block processing at various buffer sizes
// ═══════════════════════════════════════════════════════════════════

fn bench_compiled_pedals_block(c: &mut Criterion) {
    let mut group = c.benchmark_group("compiled_pedal_block");

    let pedals = [
        "tube_screamer.pedal",
        "big_muff.pedal",
        "blues_driver.pedal",
        "proco_rat.pedal",
        "klon_centaur.pedal",
    ];

    for name in pedals {
        let label = name.trim_end_matches(".pedal");
        for &block_size in BLOCK_SIZES {
            let mut proc = compile_example(name);
            let block = test_block(block_size);

            group.throughput(Throughput::Elements(block_size as u64));
            group.bench_with_input(BenchmarkId::new(label, block_size), &block, |b, block| {
                b.iter(|| {
                    for &s in block {
                        black_box(proc.process(black_box(s)));
                    }
                })
            });
        }
    }

    group.finish();
}

// ═══════════════════════════════════════════════════════════════════
// Group 4: Real-time budget validation (48 kHz / 64 samples)
// ═══════════════════════════════════════════════════════════════════

fn bench_realtime_budget(c: &mut Criterion) {
    let budget_ms = 64.0 / SAMPLE_RATE * 1_000.0; // 1.333 ms

    let mut group = c.benchmark_group("realtime_budget_48k_64");
    group.measurement_time(Duration::from_secs(5));

    let pedals = [
        "tube_screamer.pedal",
        "big_muff.pedal",
        "fuzz_face.pedal",
        "blues_driver.pedal",
        "dyna_comp.pedal",
        "klon_centaur.pedal",
        "proco_rat.pedal",
    ];

    let block = test_block(64);

    for name in pedals {
        let mut proc = compile_example(name);
        let label = name.trim_end_matches(".pedal");

        group.throughput(Throughput::Elements(64));
        group.bench_function(label, |b| {
            b.iter(|| {
                for &s in &block {
                    black_box(proc.process(black_box(s)));
                }
            })
        });
    }

    group.finish();

    eprintln!();
    eprintln!("══════════════════════════════════════════════════════");
    eprintln!("  Real-Time Budget: 48 kHz / 64 samples = {budget_ms:.3} ms");
    eprintln!("  All pedals must complete a 64-sample block within");
    eprintln!("  this budget for glitch-free real-time audio.");
    eprintln!("══════════════════════════════════════════════════════");
}

// ═══════════════════════════════════════════════════════════════════
// Group 5: Compiled vs hardcoded comparison
// ═══════════════════════════════════════════════════════════════════

fn bench_compiled_vs_hardcoded(c: &mut Criterion) {
    let mut group = c.benchmark_group("compiled_vs_hardcoded");
    group.measurement_time(Duration::from_secs(5));

    let block = test_block(256);
    group.throughput(Throughput::Elements(256));

    // Fuzz Face: compiled
    {
        let mut compiled = compile_example("fuzz_face.pedal");
        group.bench_function("fuzz_face/compiled", |b| {
            b.iter(|| {
                for &s in &block {
                    black_box(compiled.process(black_box(s)));
                }
            })
        });
    }

    group.finish();
}

// ═══════════════════════════════════════════════════════════════════
// Group 6: FLOPS estimation and throughput analysis
// ═══════════════════════════════════════════════════════════════════

/// Benchmark that estimates FLOPS for the WDF clipper.
fn bench_flops_estimation(c: &mut Criterion) {
    let mut group = c.benchmark_group("flops_estimation");
    group.measurement_time(Duration::from_secs(5));

    // WDF Clipper FLOPS estimation
    {
        let mut clipper = WdfClipper::new(4700.0, 220e-9, DiodeModel::silicon(), SAMPLE_RATE);
        let block = test_block(4096);
        let estimated_flops_per_sample = estimate_clipper_flops();

        group.throughput(Throughput::Elements(4096));
        group.bench_function("wdf_clipper_4096", |b| {
            b.iter(|| {
                for &s in &block {
                    black_box(clipper.process(black_box(s)));
                }
            })
        });

        // After benchmark, print FLOPS estimate
        eprintln!();
        eprintln!(
            "  WDF Clipper estimated FLOPs/sample: {}",
            estimated_flops_per_sample
        );
        eprintln!(
            "  At 48kHz: {} required",
            format_flops(estimated_flops_per_sample as f64 * SAMPLE_RATE)
        );
    }

    // Diode pair Newton solver (isolated)
    {
        let mut dp = DiodePairRoot::new(DiodeModel::silicon());
        let flops_per_solve = FLOPS_PER_NEWTON_ITER * AVG_NEWTON_ITERS;

        group.throughput(Throughput::Elements(10000));
        group.bench_function("diode_newton_10k", |b| {
            b.iter(|| {
                for i in 0..10000 {
                    let a = (i as f64 * 0.001).sin();
                    black_box(dp.process(black_box(a), black_box(4700.0)));
                }
            })
        });

        eprintln!();
        eprintln!("  Diode Newton solve estimated FLOPs: {}", flops_per_solve);
    }

    group.finish();
}

/// Comprehensive throughput and FLOPS report for all pedal types.
fn bench_throughput_report(c: &mut Criterion) {
    let mut group = c.benchmark_group("throughput_report");
    group.measurement_time(Duration::from_secs(3));
    group.sample_size(50);

    let pedals = [
        ("tube_screamer.pedal", 1), // 1 WDF stage
        ("big_muff.pedal", 4),      // 4 clipping stages
        ("fuzz_face.pedal", 2),     // 2 BJT stages
        ("blues_driver.pedal", 2),  // 2 stages
        ("dyna_comp.pedal", 1),     // 1 OTA stage
        ("klon_centaur.pedal", 2),  // 2 stages
        ("proco_rat.pedal", 1),     // 1 stage
        ("boss_ce2.pedal", 1),      // 1 stage + BBD + LFO
    ];

    let num_samples = 48000; // 1 second of audio
    let block = test_block(num_samples);

    eprintln!();
    eprintln!("══════════════════════════════════════════════════════════════════");
    eprintln!("  THROUGHPUT & FLOPS ANALYSIS (1 second of audio @ 48kHz)");
    eprintln!("══════════════════════════════════════════════════════════════════");
    eprintln!();
    eprintln!(
        "  {:20} {:>12} {:>14} {:>10} {:>12}",
        "Pedal", "Time (ms)", "Samples/sec", "CPU %", "Est. MFLOPS"
    );
    eprintln!(
        "  {:-<20} {:-^12} {:-^14} {:-^10} {:-^12}",
        "", "", "", "", ""
    );

    for (name, estimated_stages) in pedals {
        let mut proc = compile_example(name);
        let label = name.trim_end_matches(".pedal");

        // Warm up
        for &s in &block[..1000] {
            black_box(proc.process(black_box(s)));
        }

        // Measure
        let start = Instant::now();
        for &s in &block {
            black_box(proc.process(black_box(s)));
        }
        let elapsed = start.elapsed();

        let elapsed_ms = elapsed.as_secs_f64() * 1000.0;
        let samples_per_sec = num_samples as f64 / elapsed.as_secs_f64();
        let cpu_percent = (elapsed_ms / 1000.0) * 100.0; // Time to process 1 sec of audio

        // Rough FLOPS estimate: each stage has ~1 Newton solve + adaptor overhead
        let flops_per_sample = estimated_stages as u64 * estimate_clipper_flops();
        let mflops = (flops_per_sample as f64 * samples_per_sec) / 1e6;

        eprintln!(
            "  {:20} {:>12.2} {:>14.0} {:>9.1}% {:>12.1}",
            label, elapsed_ms, samples_per_sec, cpu_percent, mflops
        );

        // Also run through criterion for proper statistical measurement
        group.throughput(Throughput::Elements(num_samples as u64));
        group.bench_function(label, |b| {
            b.iter(|| {
                for &s in &block {
                    black_box(proc.process(black_box(s)));
                }
            })
        });
    }

    eprintln!();
    eprintln!("  Note: CPU % shows fraction of real-time budget used.");
    eprintln!("        <100% means real-time capable on this CPU.");
    eprintln!("══════════════════════════════════════════════════════════════════");

    group.finish();
}

/// Benchmark individual WDF elements in isolation.
fn bench_wdf_elements(c: &mut Criterion) {
    let mut group = c.benchmark_group("wdf_elements");
    group.measurement_time(Duration::from_secs(3));

    let num_ops = 100_000;
    group.throughput(Throughput::Elements(num_ops));

    // Series adaptor
    {
        let mut adaptor = pedalkernel::tree::SeriesAdaptor::new(1000.0, 2000.0);
        group.bench_function("series_adaptor", |b| {
            b.iter(|| {
                for i in 0..num_ops {
                    let b1 = (i as f64 * 0.00001).sin();
                    let b2 = (i as f64 * 0.00002).cos();
                    let b_up = adaptor.scatter_up(black_box(b1), black_box(b2));
                    let (a1, a2) = adaptor.scatter_down(black_box(-b_up));
                    black_box((a1, a2));
                }
            })
        });
    }

    // Parallel adaptor
    {
        let mut adaptor = pedalkernel::tree::ParallelAdaptor::new(1000.0, 2000.0);
        group.bench_function("parallel_adaptor", |b| {
            b.iter(|| {
                for i in 0..num_ops {
                    let b1 = (i as f64 * 0.00001).sin();
                    let b2 = (i as f64 * 0.00002).cos();
                    let b_up = adaptor.scatter_up(black_box(b1), black_box(b2));
                    let (a1, a2) = adaptor.scatter_down(black_box(-b_up));
                    black_box((a1, a2));
                }
            })
        });
    }

    // Capacitor
    {
        let mut cap = Capacitor::new(100e-9, SAMPLE_RATE);
        group.bench_function("capacitor", |b| {
            b.iter(|| {
                for i in 0..num_ops {
                    let incident = (i as f64 * 0.00001).sin();
                    let reflected = cap.reflected();
                    cap.set_incident(black_box(incident));
                    black_box(reflected);
                }
            })
        });
    }

    // Diode pair Newton solve
    {
        let mut dp = DiodePairRoot::new(DiodeModel::silicon());
        group.bench_function("diode_pair_newton", |b| {
            b.iter(|| {
                for i in 0..num_ops {
                    let a = (i as f64 * 0.00001).sin();
                    black_box(dp.process(black_box(a), black_box(4700.0)));
                }
            })
        });
    }

    // Single diode Newton solve
    {
        let mut d = DiodeRoot::new(DiodeModel::silicon());
        group.bench_function("single_diode_newton", |b| {
            b.iter(|| {
                for i in 0..num_ops {
                    let a = (i as f64 * 0.00001).sin();
                    black_box(d.process(black_box(a), black_box(4700.0)));
                }
            })
        });
    }

    // Germanium diode (different convergence characteristics)
    {
        let mut dp = DiodePairRoot::new(DiodeModel::germanium());
        group.bench_function("diode_pair_germanium", |b| {
            b.iter(|| {
                for i in 0..num_ops {
                    let a = (i as f64 * 0.00001).sin() * 2.0; // Higher amplitude
                    black_box(dp.process(black_box(a), black_box(4700.0)));
                }
            })
        });
    }

    // LED diode (harder to converge due to low Is)
    {
        let mut dp = DiodePairRoot::new(DiodeModel::led());
        group.bench_function("diode_pair_led", |b| {
            b.iter(|| {
                for i in 0..num_ops {
                    let a = (i as f64 * 0.00001).sin() * 5.0; // Even higher amplitude
                    black_box(dp.process(black_box(a), black_box(4700.0)));
                }
            })
        });
    }

    group.finish();

    // Print element FLOPS estimates
    eprintln!();
    eprintln!("══════════════════════════════════════════════════════════════════");
    eprintln!("  WDF ELEMENT ESTIMATED FLOPS (per operation)");
    eprintln!("══════════════════════════════════════════════════════════════════");
    eprintln!("  Series adaptor:    {} FLOPs", FLOPS_SERIES_ADAPTOR);
    eprintln!("  Parallel adaptor:  {} FLOPs", FLOPS_PARALLEL_ADAPTOR);
    eprintln!("  Capacitor:         {} FLOPs", FLOPS_CAPACITOR);
    eprintln!("  Resistor:          {} FLOPs", FLOPS_RESISTOR);
    eprintln!("  Voltage source:    {} FLOPs", FLOPS_VOLTAGE_SOURCE);
    eprintln!(
        "  Diode Newton iter: {} FLOPs × {} avg iters = {} FLOPs",
        FLOPS_PER_NEWTON_ITER,
        AVG_NEWTON_ITERS,
        FLOPS_PER_NEWTON_ITER * AVG_NEWTON_ITERS
    );
    eprintln!("══════════════════════════════════════════════════════════════════");
}

/// Scalability test: measure how performance scales with circuit complexity.
fn bench_scalability(c: &mut Criterion) {
    let mut group = c.benchmark_group("scalability");
    group.measurement_time(Duration::from_secs(3));

    // Test different numbers of cascaded clipping stages
    // This simulates circuits of increasing complexity
    let block = test_block(4096);
    group.throughput(Throughput::Elements(4096));

    eprintln!();
    eprintln!("══════════════════════════════════════════════════════════════════");
    eprintln!("  SCALABILITY: Cascaded WDF Clippers");
    eprintln!("══════════════════════════════════════════════════════════════════");

    for num_stages in [1, 2, 4, 8] {
        let mut clippers: Vec<_> = (0..num_stages)
            .map(|_| WdfClipper::new(4700.0, 220e-9, DiodeModel::silicon(), SAMPLE_RATE))
            .collect();

        let label = format!("{}_cascaded_clippers", num_stages);

        // Measure
        let start = Instant::now();
        for &s in &block {
            let mut signal = s;
            for clipper in &mut clippers {
                signal = clipper.process(signal);
            }
            black_box(signal);
        }
        let elapsed = start.elapsed();

        let ns_per_sample = elapsed.as_nanos() as f64 / 4096.0;
        let samples_per_sec = 1e9 / ns_per_sample;
        let cpu_at_48k = (SAMPLE_RATE / samples_per_sec) * 100.0;

        eprintln!(
            "  {} stages: {:.1} ns/sample, {:.1}% CPU @ 48kHz",
            num_stages, ns_per_sample, cpu_at_48k
        );

        group.bench_function(&label, |b| {
            b.iter(|| {
                for &s in &block {
                    let mut signal = s;
                    for clipper in &mut clippers {
                        signal = clipper.process(black_box(signal));
                    }
                    black_box(signal);
                }
            })
        });
    }

    eprintln!("══════════════════════════════════════════════════════════════════");

    group.finish();
}

/// Sample rate scaling: verify performance at different sample rates.
fn bench_sample_rates(c: &mut Criterion) {
    let mut group = c.benchmark_group("sample_rates");
    group.measurement_time(Duration::from_secs(3));

    let sample_rates = [44100.0, 48000.0, 96000.0, 192000.0];
    let num_samples = 48000; // ~1 second at 48kHz

    eprintln!();
    eprintln!("══════════════════════════════════════════════════════════════════");
    eprintln!("  SAMPLE RATE SCALING (Tube Screamer)");
    eprintln!("══════════════════════════════════════════════════════════════════");

    for &sr in &sample_rates {
        let mut proc = compile_example("tube_screamer.pedal");
        proc.set_sample_rate(sr);

        let block: Vec<f64> = (0..num_samples)
            .map(|i| 0.5 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / sr).sin())
            .collect();

        let label = format!("{}Hz", sr as u32);

        // Measure
        let start = Instant::now();
        for &s in &block {
            black_box(proc.process(black_box(s)));
        }
        let elapsed = start.elapsed();

        let samples_per_sec = num_samples as f64 / elapsed.as_secs_f64();
        let realtime_ratio = samples_per_sec / sr;

        eprintln!(
            "  {:>6} Hz: {:>10.0} samples/sec, {:.1}x realtime",
            sr as u32, samples_per_sec, realtime_ratio
        );

        group.throughput(Throughput::Elements(num_samples as u64));
        group.bench_function(&label, |b| {
            b.iter(|| {
                for &s in &block {
                    black_box(proc.process(black_box(s)));
                }
            })
        });
    }

    eprintln!("══════════════════════════════════════════════════════════════════");

    group.finish();
}

// ═══════════════════════════════════════════════════════════════════
// Group 7: Pro legends — per-sample and block benchmarks
// ═══════════════════════════════════════════════════════════════════

fn bench_legends_sample(c: &mut Criterion) {
    let mut group = c.benchmark_group("legends_sample");

    let legends_dir = concat!(
        env!("HOME"),
        "/src/pedalkernel/pedalkernel-pro/pedals/legends"
    );
    let pedals = [
        ("muff", "muff.pedal"),
        ("screamer", "screamer.pedal"),
        ("rangemaster", "rangemaster.pedal"),
        ("goldenrod", "goldenrod.pedal"),
        ("ratking", "ratking.pedal"),
        ("blues", "blues.pedal"),
        ("fizz", "fizz.pedal"),
    ];

    for (label, filename) in pedals {
        let path = format!("{}/{}", legends_dir, filename);
        if !std::path::Path::new(&path).exists() {
            eprintln!("  Skipping {label}: {path} not found");
            continue;
        }
        let mut proc = compile_pedal_file(&path);
        group.bench_function(label, |b| {
            let mut phase = 0.0_f64;
            b.iter(|| {
                phase += 440.0 / SAMPLE_RATE;
                let input = 0.5 * (2.0 * std::f64::consts::PI * phase).sin();
                black_box(proc.process(black_box(input)))
            })
        });
    }

    group.finish();
}

/// Benchmark with decaying signal (simulates plucked note sustain/decay).
/// This tests adaptive oversampling effectiveness: during decay, frozen Newton
/// succeeds more often → adaptive X2 kicks in → fewer NR solves.
fn bench_legends_decay(c: &mut Criterion) {
    let mut group = c.benchmark_group("legends_decay");

    let legends_dir = concat!(
        env!("HOME"),
        "/src/pedalkernel/pedalkernel-pro/pedals/legends"
    );
    let pedals = [
        ("muff", "muff.pedal"),
        ("screamer", "screamer.pedal"),
        ("rangemaster", "rangemaster.pedal"),
        ("fizz", "fizz.pedal"),
    ];

    for (label, filename) in pedals {
        let path = format!("{}/{}", legends_dir, filename);
        if !std::path::Path::new(&path).exists() {
            eprintln!("  Skipping {label}: {path} not found");
            continue;
        }
        let mut proc = compile_pedal_file(&path);
        group.bench_function(label, |b| {
            let mut phase = 0.0_f64;
            let mut sample_idx = 0u64;
            b.iter(|| {
                phase += 440.0 / SAMPLE_RATE;
                sample_idx += 1;
                // Exponential decay: -60dB over 1 second (48000 samples)
                let envelope = (-3.0 * sample_idx as f64 / SAMPLE_RATE).exp();
                let input = 0.5 * envelope * (2.0 * std::f64::consts::PI * phase).sin();
                black_box(proc.process(black_box(input)))
            })
        });
    }

    group.finish();
}

fn bench_legends_block(c: &mut Criterion) {
    let mut group = c.benchmark_group("legends_block");
    group.measurement_time(Duration::from_secs(5));

    let legends_dir = concat!(
        env!("HOME"),
        "/src/pedalkernel/pedalkernel-pro/pedals/legends"
    );
    let pedals = [
        ("muff", "muff.pedal"),
        ("screamer", "screamer.pedal"),
        ("rangemaster", "rangemaster.pedal"),
        ("goldenrod", "goldenrod.pedal"),
        ("ratking", "ratking.pedal"),
    ];

    let num_samples = 48000; // 1 second
    let block = test_block(num_samples);
    let budget_ms = num_samples as f64 / SAMPLE_RATE * 1_000.0;

    for (label, filename) in pedals {
        let path = format!("{}/{}", legends_dir, filename);
        if !std::path::Path::new(&path).exists() {
            continue;
        }
        let mut proc = compile_pedal_file(&path);

        // Warm up
        for &s in &block[..1000] {
            black_box(proc.process(black_box(s)));
        }

        // Quick manual timing for the report
        let start = Instant::now();
        for &s in &block {
            black_box(proc.process(black_box(s)));
        }
        let elapsed = start.elapsed();
        let elapsed_ms = elapsed.as_secs_f64() * 1000.0;
        let cpu_pct = (elapsed_ms / budget_ms) * 100.0;
        eprintln!("  {label:15} {elapsed_ms:8.2} ms  ({cpu_pct:5.1}% CPU @ 48kHz)");

        group.throughput(Throughput::Elements(num_samples as u64));
        group.bench_function(label, |b| {
            b.iter(|| {
                for &s in &block {
                    black_box(proc.process(black_box(s)));
                }
            })
        });
    }

    group.finish();
}

// ═══════════════════════════════════════════════════════════════════
// Group 8: Outboard EQ — Pultec EQP-1A passive+makeup
// ═══════════════════════════════════════════════════════════════════

/// Per-sample and xRT headroom bench for the Pultec EQP-1A passive+makeup chain.
///
/// The pedal compiles to 4 stages:
///   Stage 0 — WdfStage (passive RLC EQ)
///   Stage 1 — MultiNlStage (12AX7, grouped NR, bounded ≤ 24 iter)
///   Stage 2 — MultiNlStage (12AU7, grouped NR, bounded ≤ 24 iter)
///   Stage 3 — WdfStage (output coupling cap)
///
/// Real-time safety: grouped NR is bounded; no heap allocation on the hot path.
/// xRT headroom = (1 / 48_000 s) / per_sample_time_s.
fn bench_pultec_passive_makeup(c: &mut Criterion) {
    let pedal_path = concat!(
        env!("HOME"),
        "/src/pedalkernel/pedalkernel/.worktrees/bd-pedalkernel-hcpb.3",
        "/pedalkernel-validate/circuits/eq/pultec_eqp1a_passive_makeup.pedal"
    );

    if !std::path::Path::new(pedal_path).exists() {
        eprintln!("  Skipping pultec_passive_makeup: {pedal_path} not found");
        return;
    }

    let mut group = c.benchmark_group("outboard_eq");
    group.measurement_time(Duration::from_secs(5));
    group.sample_size(50);

    // ── per-sample criterion bench ────────────────────────────────────────────
    {
        let mut proc = compile_pedal_file(pedal_path);
        group.bench_function("pultec_passive_makeup/per_sample", |b| {
            let mut phase = 0.0_f64;
            b.iter(|| {
                phase += 1_000.0 / SAMPLE_RATE;
                let input = 0.1 * (2.0 * std::f64::consts::PI * phase).sin();
                black_box(proc.process(black_box(input)))
            })
        });
    }

    // ── 64-sample block criterion bench ──────────────────────────────────────
    {
        let block = test_block(64);
        let mut proc = compile_pedal_file(pedal_path);
        group.throughput(Throughput::Elements(64));
        group.bench_function("pultec_passive_makeup/block64", |b| {
            b.iter(|| {
                for &s in &block {
                    black_box(proc.process(black_box(s)));
                }
            })
        });
    }

    group.finish();

    // ── manual timing for xRT headroom report ─────────────────────────────────
    {
        let mut proc = compile_pedal_file(pedal_path);
        let num_samples: usize = 48_000;
        let block: Vec<f64> = (0..num_samples)
            .map(|i| 0.1 * (2.0 * std::f64::consts::PI * 1_000.0 * i as f64 / SAMPLE_RATE).sin())
            .collect();

        // Warm up
        for &s in &block[..1_000] {
            black_box(proc.process(black_box(s)));
        }

        let start = Instant::now();
        for &s in &block {
            black_box(proc.process(black_box(s)));
        }
        let elapsed = start.elapsed();

        let elapsed_s = elapsed.as_secs_f64();
        let per_sample_ns = elapsed_s * 1e9 / num_samples as f64;
        let budget_s = 1.0 / SAMPLE_RATE; // 20.833 µs at 48kHz
        let xrt = budget_s / (elapsed_s / num_samples as f64);

        eprintln!();
        eprintln!("══════════════════════════════════════════════════════════════════");
        eprintln!("  PULTEC EQP-1A PASSIVE+MAKEUP — REAL-TIME REPORT (48 kHz)");
        eprintln!("══════════════════════════════════════════════════════════════════");
        eprintln!("  Stages: WDF(passive EQ) → MultiNL(12AX7) → MultiNL(12AU7) → WDF(Cout)");
        eprintln!("  NL solver: grouped Newton-Raphson, bounded ≤ 24 iterations/sample");
        eprintln!("  Per-sample cost:  {per_sample_ns:.2} ns");
        eprintln!("  48kHz budget:     {:.2} µs", budget_s * 1e6);
        eprintln!("  xRT headroom:     {xrt:.1}× realtime");
        eprintln!(
            "  Status:           {}",
            if xrt >= 1.0 { "REAL-TIME SAFE" } else { "OVER BUDGET" }
        );
        eprintln!("══════════════════════════════════════════════════════════════════");
    }
}

// ═══════════════════════════════════════════════════════════════════

criterion_group!(
    existing,
    bench_wdf_clipper,
    bench_wdf_clipper_block,
    bench_diode_pair_newton
);

criterion_group!(
    compiled,
    bench_compiled_pedals_sample,
    bench_compiled_pedals_block
);

criterion_group!(realtime, bench_realtime_budget, bench_compiled_vs_hardcoded);

criterion_group!(
    flops,
    bench_flops_estimation,
    bench_throughput_report,
    bench_wdf_elements,
    bench_scalability,
    bench_sample_rates
);

criterion_group!(
    legends,
    bench_legends_sample,
    bench_legends_decay,
    bench_legends_block
);

criterion_group!(outboard_eq, bench_pultec_passive_makeup);

criterion_main!(existing, compiled, realtime, flops, legends, outboard_eq);
