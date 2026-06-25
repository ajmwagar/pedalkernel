// Temp diagnostic (pedalkernel-9xu1): inspect compiled stages + measure gain
use pedalkernel::compiler::{compile_pedal_with_options, CompileOptions};
use pedalkernel::oversampling::OversamplingFactor;
use pedalkernel::PedalProcessor;
use pedalkernel_rt::processor::Stage;

fn stage_name(s: &Stage) -> &'static str {
    match s {
        Stage::Wdf(_) => "Wdf",
        Stage::MultiNl(_) => "MultiNl",
        Stage::Iir(_) => "Iir",
        Stage::BlackFeedback(_) => "BlackFeedback",
        Stage::StateSpace(_) => "StateSpace",
        Stage::Blockwise(_) => "Blockwise",
        _ => "Other",
    }
}

fn main() {
    let path = std::env::args().nth(1).expect("circuit path arg");
    let sr = 96_000.0;
    let contents = std::fs::read_to_string(&path).unwrap();
    let def = pedalkernel::dsl::parse_pedal_file(&contents).unwrap();
    let opts = CompileOptions {
        oversampling: OversamplingFactor::X1,
        ..CompileOptions::default()
    };
    let mut pedal = compile_pedal_with_options(&def, sr, opts).unwrap();

    println!("=== {} ===", path);
    println!("num stages: {}", pedal.stages.len());
    for (i, s) in pedal.stages.iter().enumerate() {
        println!("  stage[{}]: {}", i, stage_name(s));
    }

    // Small-signal gain: tiny sine, measure peak ratio (should be ~closed-loop gain)
    for amp in [0.001_f64, 0.01, 0.05, 0.1, 0.3, 0.5, 1.0] {
        pedal.reset();
        let n = (sr * 0.05) as usize;
        let f = 1000.0;
        let mut peak_in = 0.0_f64;
        let mut peak_out = 0.0_f64;
        let mut sumsq_in = 0.0_f64;
        let mut sumsq_out = 0.0_f64;
        // warmup
        for k in 0..n {
            let x = amp * (2.0 * std::f64::consts::PI * f * k as f64 / sr).sin();
            let _ = pedal.process(x);
        }
        for k in 0..n {
            let x = amp * (2.0 * std::f64::consts::PI * f * (k + n) as f64 / sr).sin();
            let y = pedal.process(x);
            peak_in = peak_in.max(x.abs());
            peak_out = peak_out.max(y.abs());
            sumsq_in += x * x;
            sumsq_out += y * y;
        }
        let rms_in = (sumsq_in / n as f64).sqrt();
        let rms_out = (sumsq_out / n as f64).sqrt();
        println!(
            "amp={:.3}  peak_in={:.4} peak_out={:.4} (peak gain {:.2})  rms_in={:.4} rms_out={:.4} (rms gain {:.2})",
            amp, peak_in, peak_out, peak_out / peak_in, rms_in, rms_out, rms_out / rms_in
        );
    }
}
