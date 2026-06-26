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
    println!();

    // Device Q-point read from the nonlinear ROOT (the real bias point for a
    // WDF NL-root stage). This is the load-bearing section for the bias bug:
    // the passive table above is propagated through caps and does NOT see the
    // root's seeded bias or runtime-solved state.
    println!("Device Q-point (root):");
    let mut any_root = false;
    for dev in &report.devices {
        let Some(root) = &dev.root else { continue };
        any_root = true;
        // A MultiNl co-solve has no separate compile-time seed record — its
        // operating point IS the runtime-solved state — so print the seed block
        // only for genuine WDF NL roots, and label Vbe directly for the co-solve.
        let is_cosolve = root.kind.contains("co-solve");
        println!("  {} [{} root]", dev.id, root.kind);
        if is_cosolve {
            println!("    solved Vbe       {}", fmt_v(root.vbe_bias));
            println!("    solved Vce       {}", fmt_v(root.solved_vce));
        } else {
            let resolved = if root.seed_resolved {
                "Some (DC Q-point resolved)"
            } else {
                "None (UNRESOLVED -> root left at cutoff, vbe_bias=0)"
            };
            println!("    seed             {resolved}");
            println!("    seeded Vbe       {}", fmt_v(root.seeded_vbe));
            println!("    seeded Vce       {}", fmt_v(root.seeded_vce));
            println!("    vbe_bias (held)  {}", fmt_v(root.vbe_bias));
            println!("    solved Vce       {}", fmt_v(root.solved_vce));
        }
        match root.solved_ic {
            Some(i) => println!("    solved Ic        {:+8.4} mA", i * 1e3),
            None => println!("    solved Ic        {:>10}", "n/a"),
        }
    }
    // Devices that compiled WITHOUT a reachable NL root (e.g. PNP folded to
    // PassiveRType dummies) are reported explicitly so the reader does not
    // mistake an absent root for an unrecoverable value.
    for dev in &report.devices {
        if dev.root.is_none() {
            any_root = true;
            println!(
                "  {} [{}]: no nonlinear root (device folded to PassiveRType)",
                dev.id, dev.kind
            );
        }
    }
    if !any_root {
        println!("  none (no nonlinear-root devices)");
    }
}
