# Rust API Reference

PedalKernel exposes a clean Rust API for parsing, compiling, and running circuits.

## Basic usage

```rust
use pedalkernel::{dsl, compiler, kicad};

// Parse a .pedal file
let src = std::fs::read_to_string("tube_screamer.pedal")?;
let pedal = dsl::parse_pedal_file(&src)?;

// Compile to WDF
let mut proc = compiler::compile_pedal(&pedal, 48000.0)?;

// Process audio samples
proc.set_control("Drive", 0.7);
let output = proc.process(input_sample);

// Export for PCB
let netlist = kicad::export_kicad_netlist(&pedal);
```

## DSL parsing

```rust
use pedalkernel::dsl;

let src = r#"
  pedal "Tube Screamer" {
    components { ... }
    nets { ... }
    controls { ... }
  }
"#;

let pedal = dsl::parse_pedal_file(src)?;
```

The parser returns a `Pedal` struct with components, nets, and control mappings. All component values are validated during parsing.

## Compilation

```rust
use pedalkernel::compiler;

let mut proc = compiler::compile_pedal(&pedal, 48000.0)?;

// Run at a different sample rate
let mut proc_96k = compiler::compile_pedal(&pedal, 96000.0)?;
```

The compiler generates a `Processor` that can run in real-time. Each call to `.process()` does exactly one WDF solve cycle.

## Control and processing

```rust
// Set a named control
proc.set_control("Drive", 0.7);   // Range: [0.0, 1.0]

// Process one sample
let y = proc.process(x);

// Batch process
let output: Vec<f32> = input
    .iter()
    .map(|&sample| proc.process(sample))
    .collect();
```

## Hardware export (requires `hardware` feature)

```rust
use pedalkernel::hw;

// Build BOM
let bom = hw::build_bom(&pedal, None);
println!("{}", hw::format_bom_table(&pedal.name, &bom, 1));

// Check voltage safety
let warnings = hw::check_voltage_with_specs(&pedal, 18.0, &limits);
for w in warnings {
    println!("{:?}", w);
}
```

## KiCad export

```rust
use pedalkernel::kicad;

let netlist = kicad::export_kicad_netlist(&pedal);
std::fs::write("circuit.net", netlist)?;
```

The netlist is compatible with KiCad's native format. Open it in the layout tool to start designing PCBs.

## Built-in pedals (no DSL required)

For quick testing, PedalKernel includes precompiled pedal models:

```rust
// Instantiate a built-in Overdrive
let mut od = pedalkernel::pedals::Overdrive::new(48000.0);
od.set_gain(0.7);
let output = od.process(input_sample);
```

This skips parsing and compilation entirely -- useful for benchmarking or when you don't need customization.
