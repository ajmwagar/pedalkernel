# Python tools

Optional standalone scripts in `tools/`:

## Setup

```bash
python3 -m venv tools/.venv
tools/.venv/bin/pip install -r tools/requirements.txt
```

## Schematic rendering

Render EE-style schematics from `.pedal` files:

```bash
tools/.venv/bin/python tools/schematic.py examples/pedals/overdrive/tube_screamer.pedal -o ts.png
```

This generates a circuit diagram showing all components and connections.

## BOM generation

Generate Mouser bill of materials:

```bash
tools/.venv/bin/python tools/mouser_bom.py examples/pedals/fuzz/big_muff.pedal --qty 5 --csv bom.csv
```

Outputs a CSV with real Mouser part numbers, quantities, and descriptions. Upload directly to Mouser to order parts.
