# Rust API

The full API reference — every public type, function, and module — is generated from the source and lives at **[/api/pedalkernel/](/api/pedalkernel/)**. This page is a primer that points at the handful of entry points most consumers need.

## Getting a pedal from a `.pedal` file

```rust
use pedalkernel::{dsl, compiler, PedalProcessor};

let src = std::fs::read_to_string("tube_screamer.pedal")?;
let def = dsl::parse_pedal_file(&src)?;
let mut pedal = compiler::compile_pedal(&def, 48_000.0)?;
```

`dsl::parse_pedal_file` returns a `PedalDef` — the parsed AST of a `.pedal` or `.board` file. `compiler::compile_pedal` turns that AST into a runnable `CompiledPedal`. Compilation does all the graph reduction, WDF tree construction, and solver warm-up; after it returns, every `process` call runs with zero allocation.

## Processing audio

```rust
pedal.set_control("Drive", 0.7);
let y = pedal.process(x);
```

`set_control` accepts the names declared in the `controls { ... }` block of the `.pedal` file. Values are in the range each control declares. `process` consumes one input sample and returns one output sample.

## Physical realism

The default `compile_pedal` function uses sensible defaults (no oversampling, ideal component tolerances, no thermal drift). To enable any of these, compile with `compile_pedal_with_options`:

```rust
use pedalkernel::compiler::{compile_pedal_with_options, CompileOptions};
use pedalkernel::oversampling::OversamplingFactor;
use pedalkernel::tolerance::{ToleranceEngine, ToleranceGrade};

let options = CompileOptions {
    oversampling: OversamplingFactor::X4,
    tolerance: ToleranceEngine::with_grades(
        42,
        ToleranceGrade::Standard,
        ToleranceGrade::Loose,
    ),
    thermal: true,
};
let mut pedal = compile_pedal_with_options(&def, 48_000.0, options)?;
```

See the [physical realism guide](./physical-realism.md) for what each option does.

## Pedalboards

`pedalkernel::pedalboard` lets you chain multiple pedals and configure the impedance interaction between them. Interstage loading is configured on the board, not on any individual pedal:

```rust
use pedalkernel::loading::ImpedanceModel;

board.set_interstage_loading(
    0, // junction between pedal 0 and pedal 1
    ImpedanceModel::guitar_pickup(),
    ImpedanceModel::low_z_input(),
    48_000.0,
);
```

## Hardware export

With the `hardware` feature enabled:

```rust
use pedalkernel::{hw, kicad};

let bom = hw::build_bom(&def, None);
println!("{}", hw::format_bom_table(&def.name, &bom, 1));

let warnings = hw::check_voltage_with_specs(&def, 18.0, &limits);
let netlist = kicad::export_kicad_netlist(&def);
std::fs::write("circuit.net", netlist)?;
```

See [Hardware export](./hardware.md) for the larger story.

## Where to go next

- **[/api/pedalkernel/](/api/pedalkernel/)** — the full type-level reference.
- **[/api/pedalkernel_layout/](/api/pedalkernel_layout/)** — schematic layout engine.
- **[/api/pedalkernel_validate/](/api/pedalkernel_validate/)** — SPICE validation harness.
