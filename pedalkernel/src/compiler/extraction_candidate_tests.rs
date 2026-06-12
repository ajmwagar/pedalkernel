use super::spqr_build::compile_via_spqr;
use crate::PedalProcessor;

const SR: f64 = 48_000.0;

const OUTPUT_WIPER_DIVIDER: &str =
    include_str!("../../../pedalkernel-validate/circuits/extraction/output_wiper_divider.pedal");

fn sine_peak(source: &str, freq: f64, amp: f64) -> f64 {
    let pedal = crate::dsl::parse_pedal_file(source).expect("parse");
    let mut proc = compile_via_spqr(&pedal, SR).expect("compile");
    for s in 0..4000 {
        let input = amp * (std::f64::consts::TAU * freq * s as f64 / SR).sin();
        proc.process(input);
    }
    let mut peak = 0.0_f64;
    for s in 4000..4800 {
        let input = amp * (std::f64::consts::TAU * freq * s as f64 / SR).sin();
        peak = peak.max(proc.process(input).abs());
    }
    peak
}

#[test]
fn output_wiper_midpoint_matches_physical_divider() {
    let peak = sine_peak(OUTPUT_WIPER_DIVIDER, 1000.0, 1.0);
    eprintln!("output wiper midpoint peak={peak:.6}");
    assert!(
        (peak - 0.5).abs() < 0.05,
        "midpoint output wiper should be close to a 0.5x physical divider, got {peak:.6}"
    );
}
