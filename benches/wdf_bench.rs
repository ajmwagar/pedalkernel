use criterion::{black_box, criterion_group, criterion_main, Criterion};
use pedalkernel::elements::*;
use pedalkernel::tree::*;

fn bench_wdf_clipper(c: &mut Criterion) {
    let mut clipper = WdfClipper::new(4700.0, 220e-9, DiodeModel::silicon(), 48000.0);

    c.bench_function("wdf_clipper_sample", |b| {
        let mut phase = 0.0_f64;
        b.iter(|| {
            phase += 440.0 / 48000.0;
            let input = 0.5 * (2.0 * std::f64::consts::PI * phase).sin();
            black_box(clipper.process(black_box(input)))
        })
    });
}

fn bench_wdf_clipper_block(c: &mut Criterion) {
    let mut clipper = WdfClipper::new(4700.0, 220e-9, DiodeModel::silicon(), 48000.0);
    let block: Vec<f64> = (0..64)
        .map(|i| 0.5 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 48000.0).sin())
        .collect();

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

criterion_group!(benches, bench_wdf_clipper, bench_wdf_clipper_block, bench_diode_pair_newton);
criterion_main!(benches);
