use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::PedalProcessor;

fn main() {
    let pedal_name = std::env::args().nth(1).unwrap_or_else(|| "tube_screamer.pedal".to_string());
    let path = format!("{}/examples/pedals/overdrive/{}", env!("CARGO_MANIFEST_DIR"), pedal_name);
    let src = std::fs::read_to_string(&path).expect("read pedal");
    let pedal = parse_pedal_file(&src).expect("parse");
    let sr = 48000.0;
    
    use std::f64::consts::PI;
    let freq = 1000.0;
    let n = 4096;
    
    // Test both Drive and Tone
    for (ctrl, vals) in [("Drive", vec![0.0f64, 0.5, 1.0]), ("Tone", vec![0.0, 0.5, 1.0])] {
        println!("=== {} sweep ===", ctrl);
        for val in &vals {
            let mut engine = compile_pedal(&pedal, sr).expect("compile");
            // Set both at midpoint, then sweep one
            engine.set_control("Drive", if ctrl == "Drive" { *val } else { 0.5 });
            engine.set_control("Tone", if ctrl == "Tone" { *val } else { 0.5 });
            engine.set_control("Level", 0.7);
            for i in 0..8192_usize {
                let x = 0.05 * (2.0 * PI * freq * i as f64 / sr).sin();
                engine.process(x);
            }
            let mut sum_sq = 0.0;
            for i in 0..n {
                let x = 0.05 * (2.0 * PI * freq * (i + 8192) as f64 / sr).sin();
                let y = engine.process(x);
                sum_sq += y * y;
            }
            let rms = (sum_sq / n as f64).sqrt();
            println!("  {}={val:.1}: rms at {freq}Hz = {rms:.6}", ctrl);
        }
    }
}
