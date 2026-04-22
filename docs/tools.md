# Python tools & benchmarks

A handful of optional scripts live under `tools/` at the repo root. They are standalone Python — nothing in the Rust workspace depends on them.

## Setup

```bash
python3 -m venv tools/.venv
tools/.venv/bin/pip install -r tools/requirements.txt
```

## Schematic rendering

Render an EE-style schematic from a `.pedal` file using `schemdraw`:

```bash
tools/.venv/bin/python tools/schematic.py \
    pedalkernel/examples/pedals/overdrive/tube_screamer.pedal \
    -o ts.png
```

This produces a conventional circuit diagram showing every component and net. Useful for documentation, datasheets, and sanity-checking a circuit before compiling it.

## Mouser BOM CSV

Generate a bill of materials from a `.pedal` file and write it as a CSV ready to upload to Mouser:

```bash
tools/.venv/bin/python tools/mouser_bom.py \
    pedalkernel/examples/pedals/fuzz/big_muff.pedal \
    --qty 5 --csv bom.csv
```

`--qty` multiplies the per-circuit quantity by the number of units you want to build (5 units of the Big Muff in this example). The same BOM functionality is also available in Rust via `pedalkernel::hw::build_bom` — see the [Hardware export](./hardware.md) page.

## Benchmarks

PedalKernel ships a Criterion benchmark harness. Run it on your machine to get authoritative per-pedal timing:

```bash
cargo bench --bench wdf_bench
```

Output lands under `target/criterion/`. The CI workflow also runs this on every push to `main` and uploads the report as a build artifact.
