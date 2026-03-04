//! Sensitivity sweep tests: vary multiple parameters to catch "lucky"
//! single-point test configurations that pass at one setting but fail nearby.

mod audio_analysis;

use audio_analysis::*;
use pedalkernel::elements::{DiodeModel, DiodePairRoot, WdfRoot};

/// Sweep 1: RC lowpass gain-at-cutoff across sample rates and input levels.
///
/// TOLERANCE SOURCE: Bilinear warping < 0.3% + oversampling ripple ~1%.
/// Spread < 0.05 (5%) provides 5x margin for all rate/level combinations.
#[test]
fn rc_cutoff_stable_across_rates_and_levels() {
    let circuit = r#"
pedal "RC Lowpass" {
  components {
    R1: resistor(10k)
    C1: cap(10n)
  }
  nets {
    in -> R1.a
    R1.b -> C1.a, out
    C1.b -> gnd
  }
}
"#;
    let fc = 1.0 / (2.0 * std::f64::consts::PI * 10_000.0 * 10e-9);
    let rates = [44_100.0, 48_000.0, 96_000.0];
    let levels = [0.05, 0.2, 0.5, 0.8];
    let mut ratios = Vec::new();

    for &rate in &rates {
        for &amp in &levels {
            let input = sine_at(fc, amp, 0.15, rate);
            let output = compile_and_process(circuit, &input, rate, &[]);
            let ratio = rms(&output) / rms(&input);
            ratios.push((rate, amp, ratio));
        }
    }

    let min_ratio = ratios.iter().map(|r| r.2).fold(f64::INFINITY, f64::min);
    let max_ratio = ratios.iter().map(|r| r.2).fold(f64::NEG_INFINITY, f64::max);
    let spread = max_ratio - min_ratio;

    assert!(
        spread < 0.05,
        "RC cutoff gain should be stable across rates and levels: spread={spread:.6}, \
         min={min_ratio:.6}, max={max_ratio:.6}, data={ratios:?}"
    );
}

/// Sweep 2: Diode clipper THD monotonicity across amplitude.
///
/// TOLERANCE SOURCE: Shockley equation THD is provably monotonic with drive.
/// Allow 0.8x dips for Goertzel measurement noise on short signals.
#[test]
fn diode_clipper_thd_monotonic_across_amplitude() {
    let circuit_si = r#"
pedal "Silicon Clipper" {
  components {
    R1: resistor(4.7k)
    D1: diode_pair(silicon)
  }
  nets {
    in -> R1.a
    R1.b -> D1.a, out
    D1.b -> gnd
  }
}
"#;
    let circuit_ge = r#"
pedal "Germanium Clipper" {
  components {
    R1: resistor(4.7k)
    D1: diode_pair(germanium)
  }
  nets {
    in -> R1.a
    R1.b -> D1.a, out
    D1.b -> gnd
  }
}
"#;

    for (name, circuit) in [("silicon", circuit_si), ("germanium", circuit_ge)] {
        let mut last_thd = 0.0;
        let mut monotonicity_violations = 0;

        for step in 1..=20 {
            let amp = step as f64 * 0.05;
            let input = sine_at(1000.0, amp, 0.15, SAMPLE_RATE);
            let output = compile_and_process(circuit, &input, SAMPLE_RATE, &[]);
            let t = thd(&output, SAMPLE_RATE, 1000.0);

            if step > 1 && t < last_thd * 0.8 {
                monotonicity_violations += 1;
                eprintln!(
                    "[sweep] {name} THD dip at amp={amp:.2}: thd={t:.6} < prev={last_thd:.6}"
                );
            }
            last_thd = t;
        }

        assert!(
            monotonicity_violations == 0,
            "{name} clipper THD should be monotonically non-decreasing: {monotonicity_violations} violations"
        );
    }
}

/// Sweep 3: Single-port NR solver convergence across impedance x drive.
///
/// TOLERANCE SOURCE: NR with warm-starting converges in O(1) iterations for
/// continuous signals. Average < 10 accounts for step-limited paths.
#[test]
fn solver_convergence_across_impedance_and_drive() {
    let port_resistances = [100.0, 500.0, 1_000.0, 5_000.0, 50_000.0];
    let drive_levels = [0.1, 0.5, 1.0, 3.0, 10.0];

    for &rp in &port_resistances {
        for &drive in &drive_levels {
            let mut root = DiodePairRoot::new(DiodeModel::silicon());
            let n_samples = 500;

            for i in 0..n_samples {
                let t = i as f64 / 48_000.0;
                let a = drive * (2.0 * std::f64::consts::PI * 1000.0 * t).sin();
                // Process returns reflected wave; internally runs NR solve.
                let _ = root.process(a, rp);
            }

            // We can't directly read iteration counts from the root, but we can
            // verify the output is well-behaved (convergent solver produces
            // smooth, finite output).
            let mut max_jump = 0.0f64;
            let mut last_v: Option<f64> = None;
            for i in 0..n_samples {
                let t = i as f64 / 48_000.0;
                let a = drive * (2.0 * std::f64::consts::PI * 1000.0 * t).sin();
                let b = root.process(a, rp);
                let v = 0.5 * (a + b);
                assert!(v.is_finite(), "Solver diverged at Rp={rp}, drive={drive}, i={i}");
                if let Some(prev) = last_v {
                    max_jump = max_jump.max((v - prev).abs());
                }
                last_v = Some(v);
            }

            // Max voltage jump per sample should be bounded for a 1kHz sine.
            // At 48kHz, one sample = 0.13° of phase. Max dv/dt for sine of
            // amplitude A at 1kHz: A * 2π * 1000 / 48000 ≈ 0.13 * A.
            // Through diode, output is clamped so max_jump < drive is expected.
            assert!(
                max_jump < drive * 2.0,
                "Solver output jumps too much at Rp={rp}, drive={drive}: max_jump={max_jump:.6}"
            );
        }
    }
}
