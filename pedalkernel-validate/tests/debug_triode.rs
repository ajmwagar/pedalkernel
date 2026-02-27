//! Debug test: inspect WDF structure for 12AX7 triode circuit

use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::PedalProcessor;
use std::fs;

#[test]
fn debug_triode_structure() {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/circuits/nonlinear/common_cathode_12ax7.pedal"
    );
    let source = fs::read_to_string(path).expect("Failed to read pedal file");
    let def = parse_pedal_file(&source).expect("Failed to parse");

    let sample_rate = 48000.0;
    let compiled = compile_pedal(&def, sample_rate).expect("Failed to compile");

    println!("\n{}", compiled.debug_dump());

    let mut proc = compiled;

    // Test with a 1kHz sine wave
    let freq = 1000.0;
    let amplitude = 0.1; // 100mV input - reasonable for tube preamp

    println!("\n=== Triode Common Cathode Test ===");
    println!("Input: {:.0}mV sine @ {:.0}Hz", amplitude * 1000.0, freq);

    let samples_per_cycle = (sample_rate / freq) as usize;
    let mut max_out = 0.0f64;
    let mut min_out = 0.0f64;

    // Run 50 cycles to reach steady state
    for i in 0..(50 * samples_per_cycle) {
        let t = i as f64 / sample_rate;
        let input = amplitude * (2.0 * std::f64::consts::PI * freq * t).sin();
        let output = proc.process(input);

        // Track in last 20 cycles
        if i >= 30 * samples_per_cycle {
            max_out = max_out.max(output);
            min_out = min_out.min(output);
        }
    }

    let output_pp = max_out - min_out;
    let input_pp = 2.0 * amplitude;
    let gain = output_pp / input_pp;

    println!("Output max: {:.4}V", max_out);
    println!("Output min: {:.4}V", min_out);
    println!("Output p-p: {:.4}V", output_pp);
    println!("Gain: {:.1}x ({:.1} dB)", gain, 20.0 * gain.log10());

    // Expected: 12AX7 common cathode gain ~40-60
    println!("\nExpected gain: ~40-60x (32-36 dB)");
    println!("(Gain depends on plate load and operating point)");
}

#[test]
fn simple_triode_test() {
    // Minimal triode circuit
    let source = r#"
pedal "Simple Triode" {
  supply 250V {
    impedance: 50
    filter_cap: 47u
    rectifier: solid_state
  }
  components {
    V1: triode(12ax7)
    R_plate: resistor(100k)
    R_cathode: resistor(1.5k)
  }
  nets {
    in -> V1.grid
    vcc -> R_plate.a
    R_plate.b -> V1.plate, out
    V1.cathode -> R_cathode.a
    R_cathode.b -> gnd
  }
}
"#;

    let def = parse_pedal_file(source).expect("parse");
    let sample_rate = 48000.0;

    match compile_pedal(&def, sample_rate) {
        Ok(mut proc) => {
            println!("\n=== Simple Triode Compilation ===");
            println!("{}", proc.debug_dump());

            // Test with DC
            for _ in 0..1000 {
                proc.process(0.0);
            }
            let dc_out = proc.process(0.0);
            println!("\nDC output (no input): {:.4}V", dc_out);

            // Test with small AC
            let mut max_out = 0.0f64;
            for i in 0..4800 {
                let t = i as f64 / 48000.0;
                let input = 0.05 * (2.0 * std::f64::consts::PI * 1000.0 * t).sin();
                let output = proc.process(input);
                if i > 2400 {
                    max_out = max_out.max(output.abs());
                }
            }
            println!("Max output (50mV input): {:.4}V", max_out);
        }
        Err(e) => {
            println!("Compilation failed: {:?}", e);
        }
    }
}
