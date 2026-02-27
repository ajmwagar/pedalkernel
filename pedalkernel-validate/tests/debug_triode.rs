//! Debug test: inspect WDF structure for 12AX7 triode circuit

use pedalkernel::compiler::compile_pedal;
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::PedalProcessor;
use std::fs;
use std::path::Path;

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

/// Test that all triode types compile with unity pre-gain (no double-counting amplification)
#[test]
fn all_triode_types_have_unity_pregain() {
    let test_pedals_dir = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("pedalkernel/tests/test_pedals");

    let triode_files = [
        "triode_12at7.pedal",
        "triode_12ay7.pedal",
        "triode_12bh7.pedal",
        "triode_6072.pedal",
        "triode_6386.pedal",
        "triode_clean.pedal",
        "triode_ecc81.pedal",
        "triode_ecc82.pedal",
        "triode_ecc83.pedal",
        "triode_overdrive.pedal",
    ];

    println!("\n=== Testing All Triode Types for Unity Pre-Gain ===\n");

    let mut all_pass = true;
    for file in &triode_files {
        let path = test_pedals_dir.join(file);
        if !path.exists() {
            println!("  SKIP {}: file not found", file);
            continue;
        }

        let source = fs::read_to_string(&path).expect("Failed to read");
        let def = match parse_pedal_file(&source) {
            Ok(d) => d,
            Err(e) => {
                println!("  FAIL {}: parse error: {}", file, e);
                all_pass = false;
                continue;
            }
        };

        match compile_pedal(&def, 48000.0) {
            Ok(proc) => {
                let dump = proc.debug_dump();
                // Check that pre-gain is 1.0 (unity)
                if dump.contains("Pre-Gain: 1.0") {
                    println!("  PASS {}: Pre-Gain: 1.0", file);
                } else {
                    // Extract pre-gain value from dump
                    let pre_gain_line = dump.lines()
                        .find(|l| l.contains("Pre-Gain:"))
                        .unwrap_or("Pre-Gain: unknown");
                    println!("  FAIL {}: {}", file, pre_gain_line);
                    all_pass = false;
                }
            }
            Err(e) => {
                println!("  FAIL {}: compile error: {:?}", file, e);
                all_pass = false;
            }
        }
    }

    assert!(all_pass, "Some triode types did not have unity pre-gain");
}

/// Test that all pentode types compile with unity pre-gain
#[test]
fn all_pentode_types_have_unity_pregain() {
    let test_pedals_dir = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("pedalkernel/tests/test_pedals");

    let pentode_files = [
        "pentode_5881.pedal",
        "pentode_6267.pedal",
        "pentode_6550.pedal",
        "pentode_6973.pedal",
        "pentode_6aq5a.pedal",
        "pentode_6bq5.pedal",
        "pentode_6ca7.pedal",
        "pentode_6l6gc.pedal",
        "pentode_clean.pedal",
        "pentode_el84.pedal",
        "pentode_kt66.pedal",
        "pentode_kt77.pedal",
        "pentode_kt88.pedal",
        "pentode_kt90.pedal",
        "pentode_power.pedal",
    ];

    println!("\n=== Testing All Pentode Types for Unity Pre-Gain ===\n");

    let mut all_pass = true;
    for file in &pentode_files {
        let path = test_pedals_dir.join(file);
        if !path.exists() {
            println!("  SKIP {}: file not found", file);
            continue;
        }

        let source = fs::read_to_string(&path).expect("Failed to read");
        let def = match parse_pedal_file(&source) {
            Ok(d) => d,
            Err(e) => {
                println!("  FAIL {}: parse error: {}", file, e);
                all_pass = false;
                continue;
            }
        };

        match compile_pedal(&def, 48000.0) {
            Ok(proc) => {
                let dump = proc.debug_dump();
                // Check that pre-gain is 1.0 (unity)
                if dump.contains("Pre-Gain: 1.0") {
                    println!("  PASS {}: Pre-Gain: 1.0", file);
                } else {
                    // Extract pre-gain value from dump
                    let pre_gain_line = dump.lines()
                        .find(|l| l.contains("Pre-Gain:"))
                        .unwrap_or("Pre-Gain: unknown");
                    println!("  FAIL {}: {}", file, pre_gain_line);
                    all_pass = false;
                }
            }
            Err(e) => {
                println!("  FAIL {}: compile error: {:?}", file, e);
                all_pass = false;
            }
        }
    }

    assert!(all_pass, "Some pentode types did not have unity pre-gain");
}
