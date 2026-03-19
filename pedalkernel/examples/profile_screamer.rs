use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::elements::{reset_solver_stats, solver_stats_snapshot};
use pedalkernel::PedalProcessor;

fn main() {
    let pedal_name = std::env::args().nth(1).unwrap_or_else(|| "screamer".to_string());
    let path = format!(
        "{}/src/pedalkernel/pedalkernel-pro/pedals/legends/{pedal_name}.pedal",
        env!("HOME")
    );
    let src = std::fs::read_to_string(&path).expect("screamer.pedal not found");
    let pedal = parse_pedal_file(&src).unwrap();
    let mut proc = compile_pedal(&pedal, 48000.0).unwrap();

    // 10 seconds of audio at 48kHz = 480k samples
    let num_samples = 480_000;
    let mut phase = 0.0_f64;
    let mut max_out = 0.0_f64;
    let mut max_in = 0.0_f64;
    reset_solver_stats();
    for i in 0..num_samples {
        phase += 440.0 / 48000.0;
        let input = 0.5 * (2.0 * std::f64::consts::PI * phase).sin();
        let out = proc.process(input);
        max_out = max_out.max(out.abs());
        max_in = max_in.max(input.abs());
        if i < 10 || (i % 48000 == 0) {
            eprintln!("  sample {i}: in={input:.6e} out={out:.6e}");
        }
        std::hint::black_box(out);
    }
    eprintln!("max_in={max_in:.6e} max_out={max_out:.6e}");
    let stats = solver_stats_snapshot();
    let avg_iter = if stats.solves > 0 {
        stats.total_iterations as f64 / stats.solves as f64
    } else {
        0.0
    };
    eprintln!(
        "Done: {num_samples} samples, {} solves, avg {avg_iter:.2} iter/solve, max {} iter, {} iter-cap hits",
        stats.solves, stats.max_iterations, stats.iter_cap_hits
    );
}
