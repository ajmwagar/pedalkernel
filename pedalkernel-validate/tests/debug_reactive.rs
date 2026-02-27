//! Debug test: inspect WDF structure for reactive circuits

use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::PedalProcessor;
use std::fs;

#[test]
fn debug_lc_resonant() {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/circuits/reactive/lc_resonant.pedal"
    );
    let source = fs::read_to_string(path).expect("Failed to read pedal file");
    let def = parse_pedal_file(&source).expect("Failed to parse");

    let sample_rate = 48000.0;
    let compiled = compile_pedal(&def, sample_rate).expect("Failed to compile");

    println!("\n=== LC Resonant Circuit ===");
    println!("{}", compiled.debug_dump());

    let mut proc = compiled;

    // Test at resonant frequency ~1592 Hz
    let freq = 1592.0;
    let amplitude = 1.0;

    let samples_per_cycle = (sample_rate / freq) as usize;
    let mut max_out = 0.0f64;
    let mut min_out = 0.0f64;

    // Run 100 cycles to reach steady state
    for i in 0..(100 * samples_per_cycle) {
        let t = i as f64 / sample_rate;
        let input = amplitude * (2.0 * std::f64::consts::PI * freq * t).sin();
        let output = proc.process(input);

        // Track in last 20 cycles
        if i >= 80 * samples_per_cycle {
            max_out = max_out.max(output);
            min_out = min_out.min(output);
        }
    }

    let output_pp = max_out - min_out;
    let input_pp = 2.0 * amplitude;
    let gain = output_pp / input_pp;

    println!("\nAt resonant frequency {:.0} Hz:", freq);
    println!("Output p-p: {:.4}V", output_pp);
    println!("Gain: {:.3}x ({:.1} dB)", gain, 20.0 * gain.log10());
}

#[test]
fn debug_transformer_stepdown() {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/circuits/reactive/transformer_stepdown.pedal"
    );
    let source = fs::read_to_string(path).expect("Failed to read pedal file");
    let def = parse_pedal_file(&source).expect("Failed to parse");

    let sample_rate = 48000.0;
    let compiled = compile_pedal(&def, sample_rate).expect("Failed to compile");

    println!("\n=== Transformer Step-Down (10:1) ===");
    println!("{}", compiled.debug_dump());

    let mut proc = compiled;

    // Test with 1kHz sine
    let freq = 1000.0;
    let amplitude = 1.0;

    let samples_per_cycle = (sample_rate / freq) as usize;
    let mut max_out = 0.0f64;
    let mut min_out = 0.0f64;

    // Run 50 cycles
    for i in 0..(50 * samples_per_cycle) {
        let t = i as f64 / sample_rate;
        let input = amplitude * (2.0 * std::f64::consts::PI * freq * t).sin();
        let output = proc.process(input);

        if i >= 30 * samples_per_cycle {
            max_out = max_out.max(output);
            min_out = min_out.min(output);
        }
    }

    let output_pp = max_out - min_out;
    let input_pp = 2.0 * amplitude;
    let gain = output_pp / input_pp;

    println!("\nExpected gain: 0.1x (10:1 step-down)");
    println!("Actual output p-p: {:.4}V", output_pp);
    println!("Actual gain: {:.4}x ({:.1} dB)", gain, 20.0 * gain.log10());
}

#[test]
fn debug_transformer_stepup() {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/circuits/reactive/transformer_stepup.pedal"
    );
    let source = fs::read_to_string(path).expect("Failed to read pedal file");
    let def = parse_pedal_file(&source).expect("Failed to parse");

    let sample_rate = 48000.0;
    let compiled = compile_pedal(&def, sample_rate).expect("Failed to compile");

    println!("\n=== Transformer Step-Up (1:4) ===");
    println!("{}", compiled.debug_dump());

    let mut proc = compiled;

    // Test with 1kHz sine
    let freq = 1000.0;
    let amplitude = 0.25;  // Lower amplitude to avoid clipping

    let samples_per_cycle = (sample_rate / freq) as usize;
    let mut max_out = 0.0f64;
    let mut min_out = 0.0f64;

    // Run 50 cycles
    for i in 0..(50 * samples_per_cycle) {
        let t = i as f64 / sample_rate;
        let input = amplitude * (2.0 * std::f64::consts::PI * freq * t).sin();
        let output = proc.process(input);

        if i >= 30 * samples_per_cycle {
            max_out = max_out.max(output);
            min_out = min_out.min(output);
        }
    }

    let output_pp = max_out - min_out;
    let input_pp = 2.0 * amplitude;
    let gain = output_pp / input_pp;

    println!("\nExpected gain: 4.0x (1:4 step-up)");
    println!("Actual output p-p: {:.4}V", output_pp);
    println!("Actual gain: {:.4}x ({:.1} dB)", gain, 20.0 * gain.log10());
}

#[test]
fn debug_delay_simple() {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/circuits/reactive/delay_simple.pedal"
    );
    let source = fs::read_to_string(path).expect("Failed to read pedal file");
    let def = parse_pedal_file(&source).expect("Failed to parse");

    let sample_rate = 48000.0;
    let compiled = compile_pedal(&def, sample_rate).expect("Failed to compile");

    println!("\n=== Simple Delay (10ms) ===");
    println!("{}", compiled.debug_dump());

    let mut proc = compiled;

    // Test with impulse - find delay time
    let delay_samples = (0.01 * sample_rate) as usize;  // 10ms = 480 samples at 48kHz

    // Send impulse
    let first_out = proc.process(1.0);
    println!("\nFirst output (impulse in): {:.4}", first_out);

    // Read outputs looking for the delayed impulse
    let mut found_delay = 0;
    let mut max_output = 0.0f64;
    let mut max_sample = 0;
    for i in 1..1000 {
        let output = proc.process(0.0);
        if output.abs() > max_output {
            max_output = output.abs();
            max_sample = i;
        }
        if output.abs() > 0.1 && found_delay == 0 {
            found_delay = i;
            println!("Significant output at sample {}: {:.4}", i, output);
        }
    }

    println!("\nMax output: {:.4} at sample {} ({:.2}ms)",
             max_output, max_sample, max_sample as f64 / sample_rate * 1000.0);
    println!("Expected delay: {} samples ({:.2}ms)", delay_samples, 10.0);

    if found_delay == 0 {
        println!("No significant impulse detected in first 1000 samples!");
    }
}
