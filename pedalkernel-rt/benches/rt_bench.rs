//! Runtime benchmarks for pedalkernel-rt.
//!
//! Benchmarks the per-sample hot path: WDF scattering, NR solvers,
//! oversampling, adaptor math, fast math, and incremental recompute.
//! No .pedal compilation — pure runtime primitives.

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};
use std::time::Duration;

use pedalkernel_rt::dyn_node::*;
use pedalkernel_rt::elements::*;
use pedalkernel_rt::fast_math;
use pedalkernel_rt::math;
use pedalkernel_rt::oversampling::*;
use pedalkernel_rt::pot_taper::PotTaper;
use pedalkernel_rt::tree::*;
use pedalkernel_rt::wdf_leaf::*;

const SAMPLE_RATE: f64 = 48_000.0;
const NUM_OPS: u64 = 100_000;

fn test_block(size: usize) -> Vec<f64> {
    (0..size)
        .map(|i| 0.5 * math::sin(2.0 * math::PI * 440.0 * i as f64 / SAMPLE_RATE))
        .collect()
}

// ═══════════════════════════════════════════════════════════════════
// WDF Element Primitives
// ═══════════════════════════════════════════════════════════════════

fn bench_wdf_elements(c: &mut Criterion) {
    let mut group = c.benchmark_group("wdf_elements");
    group.throughput(Throughput::Elements(NUM_OPS));

    // Diode pair Newton solve (silicon)
    {
        let mut dp = DiodePairRoot::new(DiodeModel::silicon());
        group.bench_function("diode_pair_silicon", |b| {
            b.iter(|| {
                for i in 0..NUM_OPS {
                    let a = math::sin(i as f64 * 0.00001);
                    black_box(dp.process(black_box(a), black_box(4700.0)));
                }
            })
        });
    }

    // Diode pair (germanium)
    {
        let mut dp = DiodePairRoot::new(DiodeModel::germanium());
        group.bench_function("diode_pair_germanium", |b| {
            b.iter(|| {
                for i in 0..NUM_OPS {
                    let a = math::sin(i as f64 * 0.00001) * 2.0;
                    black_box(dp.process(black_box(a), black_box(4700.0)));
                }
            })
        });
    }

    // Diode pair (LED — harder convergence)
    {
        let mut dp = DiodePairRoot::new(DiodeModel::led());
        group.bench_function("diode_pair_led", |b| {
            b.iter(|| {
                for i in 0..NUM_OPS {
                    let a = math::sin(i as f64 * 0.00001) * 5.0;
                    black_box(dp.process(black_box(a), black_box(4700.0)));
                }
            })
        });
    }

    // Single diode
    {
        let mut d = DiodeRoot::new(DiodeModel::silicon());
        group.bench_function("single_diode", |b| {
            b.iter(|| {
                for i in 0..NUM_OPS {
                    let a = math::sin(i as f64 * 0.00001);
                    black_box(d.process(black_box(a), black_box(4700.0)));
                }
            })
        });
    }

    // OpAmp root (unity buffer)
    {
        let mut opamp = OpAmpRoot::unity_gain(OpAmpModel::tl072());
        opamp.set_sample_rate(SAMPLE_RATE);
        group.bench_function("opamp_unity_buffer", |b| {
            b.iter(|| {
                for i in 0..NUM_OPS {
                    opamp.set_vp(math::sin(i as f64 * 0.00001));
                    black_box(opamp.process(black_box(0.0), black_box(10_000.0)));
                }
            })
        });
    }

    group.finish();
}

// ═══════════════════════════════════════════════════════════════════
// WDF Tree Scattering
// ═══════════════════════════════════════════════════════════════════

fn bench_wdf_scattering(c: &mut Criterion) {
    let mut group = c.benchmark_group("wdf_scattering");
    group.throughput(Throughput::Elements(NUM_OPS));

    // Series adaptor scatter up + down
    {
        let mut adaptor = SeriesAdaptor::new(1000.0, 2000.0);
        group.bench_function("series_adaptor", |b| {
            b.iter(|| {
                for i in 0..NUM_OPS {
                    let b1 = math::sin(i as f64 * 0.00001);
                    let b2 = math::cos(i as f64 * 0.00002);
                    let b_up = adaptor.scatter_up(black_box(b1), black_box(b2));
                    let (a1, a2) = adaptor.scatter_down(black_box(-b_up));
                    black_box((a1, a2));
                }
            })
        });
    }

    // Parallel adaptor scatter up + down
    {
        let mut adaptor = ParallelAdaptor::new(1000.0, 2000.0);
        group.bench_function("parallel_adaptor", |b| {
            b.iter(|| {
                for i in 0..NUM_OPS {
                    let b1 = math::sin(i as f64 * 0.00001);
                    let b2 = math::cos(i as f64 * 0.00002);
                    let b_up = adaptor.scatter_up(black_box(b1), black_box(b2));
                    let (a1, a2) = adaptor.scatter_down(black_box(-b_up));
                    black_box((a1, a2));
                }
            })
        });
    }

    // DynNode tree: Series(VS, Series(R, C)) — 3-leaf passive tree
    {
        let vs = DynNode::VoltageSource(0.0, 1.0);
        let r = DynNode::Resistor(Some("R1".into()), 4700.0);
        let cap = DynNode::Capacitor(Some("C1".into()), 220e-9, 1.0 / (2.0 * SAMPLE_RATE * 220e-9));
        let inner = DynNode::Series(Box::new(r), Box::new(cap));
        let mut tree = DynNode::Series(Box::new(vs), Box::new(inner));
        tree.recompute();

        group.bench_function("dynnode_3leaf_scatter", |b| {
            b.iter(|| {
                for i in 0..NUM_OPS {
                    let input = math::sin(i as f64 * 0.00001) * 0.5;
                    tree.set_voltage(input);
                    let b_tree = tree.reflected();
                    tree.set_incident(black_box(-b_tree));
                    black_box(b_tree);
                }
            })
        });
    }

    // DynNode tree: 6-leaf (Screamer-like topology)
    {
        let vs = DynNode::VoltageSource(0.0, 1.0);
        let r1 = DynNode::Resistor(Some("R1".into()), 4700.0);
        let c1 = DynNode::Capacitor(Some("C1".into()), 47e-9, 1.0 / (2.0 * SAMPLE_RATE * 47e-9));
        let r2 = DynNode::Resistor(Some("R2".into()), 51000.0);
        let pot = DynNode::Pot("Drive".into(), 500000.0, 0.5, PotTaper::B);
        let c2 = DynNode::Capacitor(Some("C2".into()), 100e-9, 1.0 / (2.0 * SAMPLE_RATE * 100e-9));

        let branch1 = DynNode::Parallel(Box::new(r2), Box::new(pot));
        let branch2 = DynNode::Parallel(Box::new(branch1), Box::new(c1));
        let inner = DynNode::Series(Box::new(r1), Box::new(branch2));
        let with_cap = DynNode::Parallel(Box::new(inner), Box::new(c2));
        let mut tree = DynNode::Series(Box::new(vs), Box::new(with_cap));
        tree.recompute();
        tree.compute_dynamic_flags();

        group.bench_function("dynnode_6leaf_scatter", |b| {
            b.iter(|| {
                for i in 0..NUM_OPS {
                    let input = math::sin(i as f64 * 0.00001) * 0.5;
                    tree.set_voltage(input);
                    let b_tree = tree.reflected();
                    tree.set_incident(black_box(-b_tree));
                    black_box(b_tree);
                }
            })
        });
    }

    group.finish();
}

// ═══════════════════════════════════════════════════════════════════
// Incremental Recompute
// ═══════════════════════════════════════════════════════════════════

fn bench_incremental_recompute(c: &mut Criterion) {
    let mut group = c.benchmark_group("incremental_recompute");

    // Build a 6-leaf tree with one pot
    let vs = DynNode::VoltageSource(0.0, 1.0);
    let r1 = DynNode::Resistor(Some("R1".into()), 4700.0);
    let c1 = DynNode::Capacitor(Some("C1".into()), 47e-9, 1.0 / (2.0 * SAMPLE_RATE * 47e-9));
    let r2 = DynNode::Resistor(Some("R2".into()), 51000.0);
    let pot = DynNode::Pot("Drive".into(), 500000.0, 0.5, PotTaper::B);
    let c2 = DynNode::Capacitor(Some("C2".into()), 100e-9, 1.0 / (2.0 * SAMPLE_RATE * 100e-9));

    let branch1 = DynNode::Parallel(Box::new(r2), Box::new(pot));
    let branch2 = DynNode::Parallel(Box::new(branch1), Box::new(c1));
    let inner = DynNode::Series(Box::new(r1), Box::new(branch2));
    let with_cap = DynNode::Parallel(Box::new(inner), Box::new(c2));
    let mut tree_full = DynNode::Series(Box::new(vs), Box::new(with_cap));
    tree_full.recompute();

    let mut tree_incr = tree_full.clone();
    tree_incr.compute_dynamic_flags();

    let n = 10_000u64;
    group.throughput(Throughput::Elements(n));

    // Full recompute (baseline)
    group.bench_function("full_recompute", |b| {
        let mut tree = tree_full.clone();
        b.iter(|| {
            for i in 0..n {
                let pos = (i as f64 / n as f64) * 0.5 + 0.25;
                tree.set_pot("Drive", pos);
                tree.recompute();
                black_box(tree.port_resistance());
            }
        })
    });

    // Incremental recompute (what we built)
    group.bench_function("incremental_recompute", |b| {
        let mut tree = tree_incr.clone();
        b.iter(|| {
            for i in 0..n {
                let pos = (i as f64 / n as f64) * 0.5 + 0.25;
                tree.set_pot_dirty("Drive", pos);
                tree.recompute_incremental();
                black_box(tree.port_resistance());
            }
        })
    });

    group.finish();
}

// ═══════════════════════════════════════════════════════════════════
// Fast Math (LUT vs std)
// ═══════════════════════════════════════════════════════════════════

fn bench_fast_math(c: &mut Criterion) {
    let mut group = c.benchmark_group("fast_math");
    group.throughput(Throughput::Elements(NUM_OPS));

    // std exp vs LUT exp
    group.bench_function("exp_std", |b| {
        b.iter(|| {
            let mut sum = 0.0;
            for i in 0..NUM_OPS {
                let x = (i as f64 * 0.0001) - 5.0;
                sum += math::exp(black_box(x));
            }
            black_box(sum)
        })
    });

    group.bench_function("exp_lut", |b| {
        b.iter(|| {
            let mut sum = 0.0;
            for i in 0..NUM_OPS {
                let x = (i as f64 * 0.0001) - 5.0;
                sum += fast_math::fast_exp_lut(black_box(x));
            }
            black_box(sum)
        })
    });

    // std tanh vs LUT tanh
    group.bench_function("tanh_std", |b| {
        b.iter(|| {
            let mut sum = 0.0;
            for i in 0..NUM_OPS {
                let x = (i as f64 * 0.0001) - 5.0;
                sum += math::tanh(black_box(x));
            }
            black_box(sum)
        })
    });

    group.bench_function("tanh_lut", |b| {
        b.iter(|| {
            let mut sum = 0.0;
            for i in 0..NUM_OPS {
                let x = (i as f64 * 0.0001) - 5.0;
                sum += fast_math::fast_tanh_lut(black_box(x));
            }
            black_box(sum)
        })
    });

    group.finish();
}

// ═══════════════════════════════════════════════════════════════════
// Oversampler
// ═══════════════════════════════════════════════════════════════════

fn bench_oversampler(c: &mut Criterion) {
    let mut group = c.benchmark_group("oversampler");
    let block = test_block(4096);
    group.throughput(Throughput::Elements(4096));

    // X1 (passthrough)
    {
        let mut os = Oversampler::new(OversamplingFactor::X1);
        group.bench_function("x1_passthrough", |b| {
            b.iter(|| {
                for &s in &block {
                    let out = os.process(black_box(s), |x| x * 0.9);
                    black_box(out);
                }
            })
        });
    }

    // X2 (standard antialiasing)
    {
        let mut os = Oversampler::new(OversamplingFactor::X2);
        group.bench_function("x2_antialiased", |b| {
            b.iter(|| {
                for &s in &block {
                    let out = os.process(black_box(s), |x| x * 0.9);
                    black_box(out);
                }
            })
        });
    }

    // X4
    {
        let mut os = Oversampler::new(OversamplingFactor::X4);
        group.bench_function("x4_antialiased", |b| {
            b.iter(|| {
                for &s in &block {
                    let out = os.process(black_box(s), |x| x * 0.9);
                    black_box(out);
                }
            })
        });
    }

    group.finish();
}

// ═══════════════════════════════════════════════════════════════════

criterion_group!(
    elements,
    bench_wdf_elements,
);

criterion_group!(
    scattering,
    bench_wdf_scattering,
);

criterion_group!(
    recompute,
    bench_incremental_recompute,
);

criterion_group!(
    fast_math_group,
    bench_fast_math,
);

criterion_group!(
    oversampling,
    bench_oversampler,
);

criterion_main!(elements, scattering, recompute, fast_math_group, oversampling);
