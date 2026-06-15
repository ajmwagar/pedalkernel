---
title: "Hardware export"
description: "KiCad netlists, Mouser BOMs, and .pedalhw voltage-safety checks."
section: "Guide"
weight: 30
---

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

The Python tool under [`tools/mouser_bom.py`](./tools.md) reads a `.pedal` file directly and emits a CSV ready to upload to Mouser:

```bash
python3 tools/mouser_bom.py \
    pedalkernel/examples/pedals/fuzz/big_muff.pedal \
    --qty 5 --csv bom.csv
```

The mapper resolves each component to a real Mouser part from a curated database:

- **Resistors** — Yageo metal film (RC series) and carbon composition for vintage circuits
- **Capacitors** — WIMA film, Nichicon electrolytic, ceramic disc where appropriate
- **Potentiometers** — Alpha linear and audio taper
- **Transistors / op-amps / ICs** — current-production replacements for vintage parts where the originals are unavailable
- **Tubes** — JJ Electronic for common preamp and power types

There is no longer a Rust-side BOM API — the previous `pedalkernel::hw` module was removed in mid-2026 as verified-dead public surface. If you need to script BOM generation from a Rust toolchain, shell out to `mouser_bom.py` or open an issue.

## Voltage safety (`.pedalhw` files)

Running a circuit in a simulator tolerates voltages that a real build cannot. A `.pedalhw` file sits next to a `.pedal` file and declares the part numbers and voltage ratings that constrain the physical implementation:

```
# fuzz_face.pedalhw
Q1: vce_max(32) part("AC128")
Q2: vce_max(32) part("AC128")
C2: voltage_rating(16)
```

The `.pedalhw` file is consumed by the Python tooling (`mouser_bom.py` uses it for part-name overrides; `schematic.py` reads it for annotated drawings). The Rust compiler itself ships a coarser, heuristic voltage check that does not need a `.pedalhw` file:

```rust
use pedalkernel::compiler::{check_voltage_compatibility, WarningSeverity};

for w in check_voltage_compatibility(&def, 18.0) {
    println!("[{:?}] {}: {}", w.severity, w.component_id, w.message);
}
// [Danger] Q1: Germanium transistor exceeds Vce(max) at 18V (typical Ge PNP rated 15-32V)
// [Info]   V1: Tube needs 150-400V plate supply; at 18V the WDF model
//              runs fine but a physical build needs a B+ supply
```

The heuristics catch common mistakes — germanium transistors in fuzz circuits above 12 V, undersized electrolytic caps, tubes running at pedal voltages — without requiring annotation. For a strict check that uses the declared `.pedalhw` ratings, the Python tooling is the path today.
