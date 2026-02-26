//! Parse a `.pedal` file, display the AST, and export a KiCad netlist.
//!
//! Run: `cargo run --example parse_pedal`
//! Or:  `cargo run --example parse_pedal -- path/to/file.pedal`

use pedalkernel::dsl::parse_pedal_file;
use pedalkernel::kicad::export_kicad_netlist;

const DEFAULT_PEDAL: &str = r#"
# Tube Screamer TS-808 clipping stage
pedal "Tube Screamer" {
  components {
    R1: resistor(4.7k)
    C1: cap(220n)
    D1: diode_pair(silicon)
    Gain: pot(500k)
  }
  nets {
    in -> C1.a
    C1.b -> R1.a, D1.a
    D1.b -> gnd
    R1.b -> Gain.a
    Gain.b -> out
  }
  controls {
    Gain.position -> "Drive" [0.0, 1.0] = 0.5
  }
}
"#;

fn main() {
    let args: Vec<String> = std::env::args().collect();

    let source = if args.len() > 1 {
        std::fs::read_to_string(&args[1]).unwrap_or_else(|e| {
            eprintln!("Error reading {}: {}", args[1], e);
            std::process::exit(1);
        })
    } else {
        println!("No .pedal file specified, using built-in Tube Screamer example.");
        println!("Usage: cargo run --example parse_pedal -- <file.pedal>\n");
        DEFAULT_PEDAL.to_string()
    };

    // Parse
    let pedal = match parse_pedal_file(&source) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("Parse error: {e}");
            std::process::exit(1);
        }
    };

    // Display AST
    println!("Pedal: \"{}\"", pedal.name);
    println!();

    println!("Components ({}):", pedal.components.len());
    for c in &pedal.components {
        println!("  {}: {:?}", c.id, c.kind);
    }
    println!();

    println!("Nets ({}):", pedal.nets.len());
    for n in &pedal.nets {
        let to_strs: Vec<String> = n.to.iter().map(|p| format!("{:?}", p)).collect();
        println!("  {:?} -> {}", n.from, to_strs.join(", "));
    }
    println!();

    if !pedal.controls.is_empty() {
        println!("Controls ({}):", pedal.controls.len());
        for c in &pedal.controls {
            println!(
                "  {}.{} -> \"{}\" [{}, {}] = {}",
                c.component, c.property, c.label, c.range.0, c.range.1, c.default
            );
        }
        println!();
    }

    // KiCad export
    let netlist = export_kicad_netlist(&pedal);
    let net_path = format!("{}.net", pedal.name.to_lowercase().replace(' ', "_"));
    std::fs::write(&net_path, &netlist).unwrap();
    println!("KiCad netlist written → {net_path}");
    println!();
    println!("--- Netlist preview ---");
    // Show first 20 lines
    for line in netlist.lines().take(20) {
        println!("{line}");
    }
    if netlist.lines().count() > 20 {
        println!("  ...");
    }
}
