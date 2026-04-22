# Hardware Export

A `.pedal` file is not just a simulation target. The same description drives KiCad netlists, bills of material, and voltage-safety checks, so you can go from compiled tone to a physical build without re-entering the circuit anywhere.

## The pipeline

```
                      +---> WDF audio engine   (WAV / JACK real-time)
                      |
  .pedal file  ------>+---> KiCad netlist       (PCB layout)
                      |
                      +---> Bill of Materials   (Mouser-ready)
```

All three outputs are generated from the same parsed `PedalDef`. Components, values, and nets are preserved exactly — there is no translation layer.

## KiCad export

Every component in the DSL maps to a real KiCad symbol. Triodes route to the correct `Valve:ECC8x` variants, power pentodes to `Valve:6L6GC` / `Valve:EL34` / `Valve:6550`, JFETs to `Device:Q_NJFET_DGS`, and transformers to the matching `Transformer:*` symbol based on winding type. Nets are preserved by name.

```rust
use pedalkernel::kicad;

let netlist = kicad::export_kicad_netlist(&def);
std::fs::write("circuit.net", netlist)?;
```

Open the `.net` file in KiCad and start laying out copper.

Schematic-level export (positioned symbols rather than just connectivity) lives in a sibling crate, `pedalkernel-layout`, which handles automatic placement, layering, and routing.

## Bill of materials

The BOM engine maps each component to a real Mouser part number from a curated database:

- **Resistors** — Yageo metal film (RC series) and carbon composition for vintage circuits
- **Capacitors** — WIMA film, Nichicon electrolytic, ceramic disc where appropriate
- **Potentiometers** — Alpha linear and audio taper
- **Transistors / op-amps / ICs** — current-production replacements for vintage parts where the originals are unavailable
- **Tubes** — JJ Electronic for common preamp and power types

```rust
use pedalkernel::hw;

let bom = hw::build_bom(&def, None);
print!("{}", hw::format_bom_table(&def.name, &bom, 1));
```

The `hardware` feature must be enabled in `Cargo.toml` to use `pedalkernel::hw`.

There is also a Python tool under [`tools/mouser_bom.py`](./tools.md) that reads a `.pedal` file directly and emits a CSV ready to upload to Mouser.

## Voltage safety (`.pedalhw` files)

Running a circuit in a simulator tolerates voltages that a real build cannot. A `.pedalhw` file sits next to a `.pedal` file and declares the part numbers and voltage ratings that constrain the physical implementation:

```
# fuzz_face.pedalhw
Q1: vce_max(32) part("AC128")
Q2: vce_max(32) part("AC128")
C2: voltage_rating(16)
```

The voltage checker reads this against a planned supply rail and returns warnings:

```rust
use pedalkernel::hw;

let warnings = hw::check_voltage_with_specs(&def, 18.0, &limits);
// [Danger] Q1: Germanium transistor exceeds Vce(max) 32V at 18V
// [Info]   V1: Tube needs 150-400V plate supply; at 18V the WDF model
//              runs fine but a physical build needs a B+ supply
```

Without a `.pedalhw` file, heuristic checks still catch common mistakes: germanium transistors in fuzz circuits above 12 V, undersized electrolytic caps, tubes running at pedal voltages.
