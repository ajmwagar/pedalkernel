//! Debug test: inspect WDF structure for single diode circuit

use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::PedalProcessor;
use std::fs;

#[test]
fn debug_single_diode_structure() {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/circuits/nonlinear/single_diode.pedal"
    );
    let source = fs::read_to_string(path).expect("Failed to read single_diode.pedal");
    let def = parse_pedal_file(&source).expect("Failed to parse");

    let sample_rate = 48000.0;
    let compiled = compile_pedal(&def, sample_rate).expect("Failed to compile");

    println!("\n{}", compiled.debug_dump());

    let mut proc = compiled;

    // Test with a sine wave - 1kHz, 1V amplitude
    let freq = 1000.0;
    let amplitude = 1.0;

    println!("\n=== Single Diode Half-Wave Rectifier ===");
    println!("Input: {:.1}V sine @ {:.0}Hz", amplitude, freq);

    // For a half-wave rectifier with R1=2.2k, RL=10k, diode to ground:
    // Positive half: diode conducts when Vin > Vf (~0.6V)
    //   Output = Vin - Vf (through R1||RL divider)
    // Negative half: diode blocks
    //   Output = Vin * RL/(R1+RL) = Vin * 0.82

    // Run for several cycles
    let samples_per_cycle = (sample_rate / freq) as usize;
    let mut positive_peak = 0.0f64;
    let mut negative_peak = 0.0f64;

    // Run 20 cycles to reach steady state
    for i in 0..(20 * samples_per_cycle) {
        let t = i as f64 / sample_rate;
        let input = amplitude * (2.0 * std::f64::consts::PI * freq * t).sin();
        let output = proc.process(input);

        // Track peaks in last 10 cycles
        if i >= 10 * samples_per_cycle {
            if output > positive_peak {
                positive_peak = output;
            }
            if output < negative_peak {
                negative_peak = output;
            }
        }
    }

    println!("Positive peak: {:.4}V", positive_peak);
    println!("Negative peak: {:.4}V", negative_peak);

    // Expected for 1V input:
    // Positive: ~0.4V (1V - 0.6V Vf, attenuated)
    // Negative: ~-0.82V (passes through divider, diode blocks)
    println!("\nExpected (approximate):");
    println!("  Positive peak: ~0.3-0.4V (Vin - Vf, attenuated by R1)");
    println!("  Negative peak: ~-0.82V (diode blocks, R divider)");

    // Check asymmetry - that's the key behavior
    let asymmetry = positive_peak.abs() / negative_peak.abs();
    println!("\nAsymmetry ratio (pos/neg): {:.3}", asymmetry);
    println!("(Should be < 1 since positive is clipped by diode)");
}

#[test]
fn compare_single_vs_pair() {
    // Compare behavior of single diode vs diode pair
    // Use MINIMAL circuit: just R + diode to ground
    let single_source = r#"
pedal "Single" {
  components {
    R1: resistor(2.2k)
    D1: diode(silicon)
  }
  nets {
    in -> R1.a
    R1.b -> D1.anode, out
    D1.cathode -> gnd
  }
}
"#;

    let pair_source = r#"
pedal "Pair" {
  components {
    R1: resistor(2.2k)
    D1: diode_pair(silicon)
  }
  nets {
    in -> R1.a
    R1.b -> D1.a, out
    D1.b -> gnd
  }
}
"#;

    println!("\n=== Parsing Single Diode ===");
    let single_def = parse_pedal_file(single_source).expect("parse single");
    for comp in &single_def.components {
        println!("  Component: {} = {:?}", comp.id, comp.kind);
    }

    println!("\n=== Parsing Diode Pair ===");
    let pair_def = parse_pedal_file(pair_source).expect("parse pair");
    for comp in &pair_def.components {
        println!("  Component: {} = {:?}", comp.id, comp.kind);
    }

    let sample_rate = 48000.0;

    println!("\n=== Compiling Single Diode ===");
    let mut single_proc = compile_pedal(&single_def, sample_rate).expect("compile single");
    println!("{}", single_proc.debug_dump());

    println!("\n=== Compiling Diode Pair ===");
    let mut pair_proc = compile_pedal(&pair_def, sample_rate).expect("compile pair");
    println!("{}", pair_proc.debug_dump());

    // Test with DC inputs
    println!("\n=== DC Response Comparison ===");
    println!("{:>10} {:>12} {:>12}", "Input", "Single Out", "Pair Out");

    for input in [-1.0, -0.5, 0.0, 0.5, 1.0].iter() {
        // Process several samples to reach steady state
        for _ in 0..100 {
            single_proc.process(*input);
            pair_proc.process(*input);
        }
        let single_out = single_proc.process(*input);
        let pair_out = pair_proc.process(*input);

        println!("{:>10.2}V {:>12.4}V {:>12.4}V", input, single_out, pair_out);
    }
}
