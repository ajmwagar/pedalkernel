//! Property-style adaptor validation: random wave inputs must satisfy KCL/KVL.

use pedalkernel::tree::{ParallelAdaptor, SeriesAdaptor};
use rand::{rngs::StdRng, Rng, SeedableRng};

#[test]
fn series_adaptor_obeys_kcl_kvl() {
    let mut rng = StdRng::seed_from_u64(0x4590_1234);
    for case in 0..256 {
        let r1 = rng.gen_range(10.0..50_000.0);
        let r2 = rng.gen_range(10.0..50_000.0);
        let mut adaptor = SeriesAdaptor::new(r1, r2);
        let b1 = rng.gen_range(-5.0..5.0);
        let b2 = rng.gen_range(-5.0..5.0);
        let expected_b3 = -(b1 + b2);
        let b3 = adaptor.scatter_up(b1, b2);
        assert!(
            (b3 - expected_b3).abs() < 1e-9,
            "Series scatter_up mismatch (case {case})"
        );
        let a3 = rng.gen_range(-5.0..5.0);
        let (a1, a2) = adaptor.scatter_down(a3);
        let gamma = r1 / (r1 + r2);
        let sum = b1 + b2 + a3;
        let expected_a1 = b1 - gamma * sum;
        let expected_a2 = b2 - (1.0 - gamma) * sum;
        assert!(
            (a1 - expected_a1).abs() < 1e-9,
            "Series scatter_down a1 mismatch (case {case})"
        );
        assert!(
            (a2 - expected_a2).abs() < 1e-9,
            "Series scatter_down a2 mismatch (case {case})"
        );
    }
}

#[test]
fn parallel_adaptor_obeys_kcl_kvl() {
    let mut rng = StdRng::seed_from_u64(0x9249_abcd);
    for case in 0..256 {
        let r1 = rng.gen_range(10.0..50_000.0);
        let r2 = rng.gen_range(10.0..50_000.0);
        let mut adaptor = ParallelAdaptor::new(r1, r2);
        let b1 = rng.gen_range(-5.0..5.0);
        let b2 = rng.gen_range(-5.0..5.0);
        let gamma = r2 / (r1 + r2);
        let expected_b3 = b1 + gamma * (b2 - b1);
        let b3 = adaptor.scatter_up(b1, b2);
        assert!(
            (b3 - expected_b3).abs() < 1e-9,
            "Parallel scatter_up mismatch (case {case})"
        );
        let a3 = rng.gen_range(-5.0..5.0);
        let (a1, a2) = adaptor.scatter_down(a3);
        let diff = b2 - b1;
        let expected_a1 = a3 + (1.0 - gamma) * diff;
        let expected_a2 = a3 - gamma * diff;
        assert!(
            (a1 - expected_a1).abs() < 1e-9,
            "Parallel scatter_down a1 mismatch (case {case})"
        );
        assert!(
            (a2 - expected_a2).abs() < 1e-9,
            "Parallel scatter_down a2 mismatch (case {case})"
        );
    }
}
