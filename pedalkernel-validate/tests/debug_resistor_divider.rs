//! Debug test: inspect WDF structure for resistor divider
//!
//! This test compiles the simplest possible circuit (10k/10k resistor divider)
//! and dumps its structure to identify why gain is wrong.

use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::PedalProcessor;
use std::fs;

#[test]
fn debug_resistor_divider_structure() {
    // Load and parse the circuit
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/circuits/linear/resistor_divider.pedal"
    );
    let source = fs::read_to_string(path).expect("Failed to read resistor_divider.pedal");
    let def = parse_pedal_file(&source).expect("Failed to parse");

    // Compile at 48kHz
    let sample_rate = 48000.0;
    let compiled = compile_pedal(&def, sample_rate).expect("Failed to compile");

    // Dump structure
    println!("\n{}", compiled.debug_dump());

    // Manual validation: process a few samples with known input
    let mut proc = compiled;

    // DC input of 1.0V
    // Expected output: 1.0 * (10k/(10k+10k)) = 0.5V = -6.02 dB attenuation
    let input = 1.0;

    // Process a few samples to reach steady state
    for _ in 0..100 {
        let _ = proc.process(input);
    }

    // Measure output
    let output = proc.process(input);

    println!("\n=== Signal Flow Debug ===");
    println!("Input:  {:.6}V", input);
    println!("Output: {:.6}V", output);
    println!("Gain:   {:.6} ({:.2} dB)", output / input, 20.0 * (output / input).log10());
    println!("Expected: 0.5 (-6.02 dB)");
    println!("Error: {:.2} dB", 20.0 * ((output / input) / 0.5).log10());

    // This test intentionally doesn't assert - it's for inspection
    // If gain is wrong, the debug_dump above should reveal why
}

#[test]
fn debug_rc_lowpass_structure() {
    // Also dump RC lowpass to see reactive element handling
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/circuits/linear/rc_lowpass.pedal"
    );
    let source = fs::read_to_string(path).expect("Failed to read rc_lowpass.pedal");
    let def = parse_pedal_file(&source).expect("Failed to parse");

    let sample_rate = 48000.0;
    let compiled = compile_pedal(&def, sample_rate).expect("Failed to compile");

    println!("\n{}", compiled.debug_dump());

    // Process 1kHz sine at steady state
    let mut proc = compiled;
    let freq = 1000.0;
    let fc = 1.0 / (2.0 * std::f64::consts::PI * 10_000.0 * 10e-9); // ~1592 Hz

    // Expected gain at 1kHz: 1/sqrt(1 + (f/fc)^2) = ~0.847 = -1.44 dB
    let expected_gain = 1.0 / (1.0 + (freq / fc).powi(2)).sqrt();

    // Process for 10 cycles to reach steady state
    let samples_per_cycle = (sample_rate / freq) as usize;
    let mut max_out = 0.0f64;
    let input_amp = 1.0;

    for i in 0..(10 * samples_per_cycle) {
        let t = i as f64 / sample_rate;
        let input = input_amp * (2.0 * std::f64::consts::PI * freq * t).sin();
        let output = proc.process(input);
        max_out = max_out.max(output.abs());
    }

    println!("\n=== RC Lowpass @ 1kHz ===");
    println!("Cutoff: {:.1} Hz", fc);
    println!("Test freq: {:.0} Hz", freq);
    println!("Expected gain: {:.4} ({:.2} dB)", expected_gain, 20.0 * expected_gain.log10());
    println!("Actual gain:   {:.4} ({:.2} dB)", max_out, 20.0 * max_out.log10());
    println!("Error: {:.2} dB", 20.0 * (max_out / expected_gain).log10());
}

#[test]
fn trace_wave_variables() {
    // Trace wave variables through resistor divider
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/circuits/linear/resistor_divider.pedal"
    );
    let source = fs::read_to_string(path).expect("Failed to read resistor_divider.pedal");
    let def = parse_pedal_file(&source).expect("Failed to parse");

    let sample_rate = 48000.0;
    let compiled = compile_pedal(&def, sample_rate).expect("Failed to compile");

    println!("\n{}", compiled.debug_dump());

    // For a resistor divider:
    // Topology: input -> R1 -> (junction: output) -> R2 -> ground
    //
    // Expected WDF tree:
    //   Parallel(R1, R2) with input as voltage source
    //   OR Series(Vsrc, Parallel(R1, R2))
    //
    // Key relationships:
    //   - Wave variable a = V + I*Rp (incident)
    //   - Wave variable b = V - I*Rp (reflected)
    //   - Voltage V = (a + b) / 2
    //   - Current I = (a - b) / (2*Rp)
    //
    // For a resistor leaf: b = 0 (no reflection from pure resistance)
    // For parallel adaptor: Rp = R1*R2/(R1+R2), gamma = R2/(R1+R2)
    //   scatter_up: b_out = b1 + gamma*(b2-b1)
    //   For R1=R2=10k: Rp=5k, gamma=0.5
    //   If both b1=b2=0: b_out = 0
    //
    // The output voltage should be Vin * R2/(R1+R2) = Vin * 0.5
    //
    // Something must be wrong in:
    // 1. Tree construction (wrong topology?)
    // 2. Input injection (voltage source handling?)
    // 3. Output extraction (where is voltage read from?)
    // 4. Gain compensation (pre_gain or output_gain wrong?)

    println!("\n=== Wave Variable Theory ===");
    println!("For R1=R2=10k parallel:");
    println!("  Rp = R1*R2/(R1+R2) = 5000 ohms");
    println!("  gamma = R2/(R1+R2) = 0.5");
    println!("  Resistor reflected wave b = 0 (always)");
    println!("  Parallel scatter_up: b_out = b1 + gamma*(b2-b1) = 0");
    println!("");
    println!("Question: How does input voltage get into the tree?");
    println!("Question: How is output voltage extracted from waves?");
}
