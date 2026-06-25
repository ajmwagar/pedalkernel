use pedalkernel::compiler::{compile_pedal, compile_pedal_with_options, CompileOptions};
use pedalkernel::dsl::parse_pedal_file;
use pedalkernel_rt::operating_point::OperatingPoint;
use std::process;

pub fn run(file_path: &str, op: bool) {
    // Parse the .pedal file
    let source = std::fs::read_to_string(file_path).unwrap_or_else(|e| {
        eprintln!("Error reading {file_path}: {e}");
        process::exit(1);
    });

    let pedal = parse_pedal_file(&source).unwrap_or_else(|e| {
        eprintln!("Parse error: {e}");
        process::exit(1);
    });

    // Print parsed component summary
    println!("file: {file_path}");
    println!("name: {}", pedal.name);
    if let Some(ref sub) = pedal.subtitle {
        println!("subtitle: {sub}");
    }
    println!("components: {}", pedal.components.len());
    println!("nets: {}", pedal.nets.len());
    println!();

    // Compile (with operating-point computation when --op is requested).
    let compiled = if op {
        let options = CompileOptions {
            compute_operating_point: true,
            ..CompileOptions::default()
        };
        match compile_pedal_with_options(&pedal, 48000.0, options) {
            Ok(c) => c,
            Err(e) => {
                eprintln!("Compile error: {e}");
                process::exit(1);
            }
        }
    } else {
        match compile_pedal(&pedal, 48000.0) {
            Ok(c) => c,
            Err(e) => {
                eprintln!("Compile error: {e}");
                process::exit(1);
            }
        }
    };

    if op {
        match &compiled.operating_point {
            Some(report) => print_operating_point(report, compiled.supply_voltage()),
            None => println!("operating point: <not computed>"),
        }
    } else {
        // Print the standard debug dump (unchanged default behavior).
        print!("{}", compiled.debug_dump());
    }
}

fn fmt_v(v: Option<f64>) -> String {
    match v {
        Some(v) => format!("{v:+8.4} V"),
        None => format!("{:>10}", "n/a"),
    }
}

fn print_operating_point(report: &OperatingPoint, supply: f64) {
    println!("=== DC Operating Point ===");
    println!(
        "supply: {supply:.3} V   settled: {} samples (~{:.0} ms @ 48 kHz)",
        report.settle_samples,
        report.settle_samples as f64 / 48.0
    );
    println!();

    // Per-net voltage table.
    println!("Nets (settled DC voltage):");
    println!("  {:<22}{:>12}   {}", "net", "voltage", "source");
    println!("  {}", "-".repeat(48));
    for net in &report.nets {
        println!(
            "  {:<22}{:>12}   {}",
            net.name,
            fmt_v(net.voltage),
            net.source.label()
        );
    }
    println!();

    // Per-device Q-point table.
    if report.devices.is_empty() {
        println!("Devices: none (no transistors found)");
    } else {
        println!("Devices (Q-point):");
        for dev in &report.devices {
            println!("  {} [{}]", dev.id, dev.kind);
            for (pin, v) in &dev.terminals {
                println!("    {:<10}{}", pin, fmt_v(*v));
            }
            let ic_label = if dev.kind == "njfet"
                || dev.kind == "pjfet"
                || dev.kind == "nmos"
                || dev.kind == "pmos"
            {
                "Id"
            } else {
                "Ic"
            };
            match dev.ic {
                Some(i) => println!("    {:<10}{:+8.4} mA", ic_label, i * 1e3),
                None => println!("    {:<10}{:>10}", ic_label, "n/a"),
            }
        }
    }
}
