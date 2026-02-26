use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};
use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::elements::*;
use pedalkernel::pedals::{FuzzFace, Overdrive};
use pedalkernel::tree::*;
use pedalkernel::PedalProcessor;
use std::time::Duration;

const SAMPLE_RATE: f64 = 48_000.0;
const BLOCK_SIZES: &[usize] = &[64, 128, 256, 512];

fn test_block(size: usize) -> Vec<f64> {
    (0..size)
        .map(|i| 0.5 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / SAMPLE_RATE).sin())
        .collect()
}

fn compile_example(name: &str) -> Box<dyn PedalProcessor> {
    let path = find_example(name);
    let src = std::fs::read_to_string(&path).unwrap();
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
    let dp = DiodePairRoot::new(DiodeModel::silicon());

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

    // Hardcoded processors for comparison.
    let mut od = Overdrive::new(SAMPLE_RATE);
    od.set_gain(0.7);
    group.throughput(Throughput::Elements(64));
    group.bench_function("hardcoded_overdrive", |b| {
        b.iter(|| {
            for &s in &block {
                black_box(od.process(black_box(s)));
            }
        })
    });

    let mut ff = FuzzFace::new(SAMPLE_RATE);
    ff.set_fuzz(0.8);
    group.throughput(Throughput::Elements(64));
    group.bench_function("hardcoded_fuzzface", |b| {
        b.iter(|| {
            for &s in &block {
                black_box(ff.process(black_box(s)));
            }
        })
    });

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

    // Tube Screamer: compiled vs hardcoded Overdrive
    {
        let mut compiled = compile_example("tube_screamer.pedal");
        group.bench_function("tube_screamer/compiled", |b| {
            b.iter(|| {
                for &s in &block {
                    black_box(compiled.process(black_box(s)));
                }
            })
        });

        let mut hardcoded = Overdrive::new(SAMPLE_RATE);
        hardcoded.set_gain(0.5);
        group.bench_function("tube_screamer/hardcoded", |b| {
            b.iter(|| {
                for &s in &block {
                    black_box(hardcoded.process(black_box(s)));
                }
            })
        });
    }

    // Fuzz Face: compiled vs hardcoded FuzzFace
    {
        let mut compiled = compile_example("fuzz_face.pedal");
        group.bench_function("fuzz_face/compiled", |b| {
            b.iter(|| {
                for &s in &block {
                    black_box(compiled.process(black_box(s)));
                }
            })
        });

        let mut hardcoded = FuzzFace::new(SAMPLE_RATE);
        hardcoded.set_fuzz(0.7);
        group.bench_function("fuzz_face/hardcoded", |b| {
            b.iter(|| {
                for &s in &block {
                    black_box(hardcoded.process(black_box(s)));
                }
            })
        });
    }

    group.finish();
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

criterion_main!(existing, compiled, realtime);
