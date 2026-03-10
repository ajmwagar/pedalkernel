# Hardware Export & PCB Design

PedalKernel can export your circuit for physical PCB layout and generate a bill of materials. This document explains the hardware pipeline.

## From tone to PCB

The same `.pedal` file drives three outputs:

```
                          +---> WDF audio engine ---> WAV / JACK real-time
                          |
  .pedal file ---> parse -+---> KiCad netlist ---> PCB layout
                          |
                          +---> Bill of Materials ---> Mouser order
```

## KiCad export

Every component maps to a real KiCad symbol:

- **Triodes**: `Valve:ECC83` / `ECC81` / `ECC82` (maps by model)
- **Power pentodes**: `Valve:6L6GC` / `Valve:EL34` / `Valve:6550` (by model)
- **JFETs**: `Device:Q_NJFET_DGS`
- **Transformers**: `Transformer:Transformer_1P_CT_1S_CT`

Nets are preserved exactly. Open the `.net` file in KiCad and start laying out copper.

### Example: Export Klon Centaur

```bash
cargo run --example parse_pedal -- examples/pedals/overdrive/klon_centaur.pedal
```

This generates a `.net` file with all symbols and connections ready for PCB layout.

## Bill of materials

The BOM engine maps your circuit to real parts from a curated database:

- Yageo metal film resistors
- WIMA film caps
- Nichicon electrolytics
- Alpha pots
- JJ Electronic tubes

Generate a BOM as a Rust program:

```rust
let bom = pedalkernel::hw::build_bom(&pedal, None);
print!("{}", pedalkernel::hw::format_bom_table(&pedal.name, &bom, 1));
// -> Mouser P/Ns, quantities, descriptions, ready to order
```

## Hardware specs (`.pedalhw` files)

For builders who need to know if a part will survive the voltage. Declare real specs alongside your circuit:

```
# fuzz_face.pedalhw
Q1: vce_max(32) part("AC128")
Q2: vce_max(32) part("AC128")
C2: voltage_rating(16)
```

Run voltage compatibility checks before you power anything up:

```rust
let warnings = pedalkernel::hw::check_voltage_with_specs(&pedal, 18.0, &limits);
// [Danger] Q1: Germanium transistor exceeds Vce(max) 32V at 18V
// [Info]   V1: Tube needs 150-400V plate supply; at 18V the WDF model
//              runs fine but a physical build needs a B+ supply
```

Without a `.pedalhw` file, heuristic checks still catch obvious problems:

- Germanium transistors in fuzz circuits above 12V
- Undersized electrolytic caps
- Tubes at pedal voltages

## Context: `.pedalhw` vs UI manifests

Note on file types:

- **`.pedal`** is **circuit-only** (signal path + parameter mapping to pots/switches).
- **`.pedalhw`** is **hardware/skin metadata** (e.g., parts specs, faceplate/finish in the pro toolchain).
- **UI manifests** (JSON) are in `pedalkernel-vst/ui/` for non-secret control-surface metadata (labels, knob/switch list, enclosure preset).

The pro pipeline may merge `.pedalhw` + UI manifest + internal assets when generating final branded skins.
