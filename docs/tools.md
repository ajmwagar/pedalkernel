---
title: "CLI & tools"
description: "Python scripts for schematics and BOMs, plus cargo bench."
section: "Reference"
weight: 80
---

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

`--qty` multiplies the per-circuit quantity by the number of units you want to build (5 units of the Big Muff in this example). The tool reads `.pedal` files directly and picks up part-name overrides from any companion `.pedalhw` file. See the [Hardware export](./hardware.md) page for the broader story.

## Hardware FX-loop capture

`pedalkernel-fx-loop` is a cross-platform Rust utility for capturing real hardware references through an audio interface. It uses `cpal`, so the same binary targets CoreAudio, WASAPI, ALSA, and JACK backends where supported.

List visible audio devices:

```bash
cargo run -p pedalkernel-fx-loop -- list
```

Capture a Scarlett 4i4 FX insert with output 3 feeding a pedal and input 3 recording the return:

```bash
cargo run -p pedalkernel-fx-loop -- capture \
    --device Scarlett \
    --send-channel 3 \
    --return-channel 3 \
    --signal sine \
    --freq 1000 \
    --duration 4 \
    --amp 0.15 \
    --out-dir hardware-goldens/ts9_noon \
    --label sine_1k
```

Artifacts are written as `*.stimulus.wav`, `*.return.wav`, `*.send_return.wav`, `*.stimulus.npy`, `*.return.npy`, and `*.json`. Channel numbers are 1-based to match the labels on the interface.

For realtime checking, loop a signal through the same insert and mirror the return to monitor outputs 1 and 2:

```bash
cargo run -p pedalkernel-fx-loop -- monitor \
    --device Scarlett \
    --send-channel 3 \
    --return-channel 3 \
    --monitor-channels 1,2 \
    --monitor-gain 0.5 \
    --signal sweep \
    --duration 8 \
    --run-duration 30
```

Use `--signal wav --input-wav path/to/file.wav` to play a guitar or reamp file instead of generated sine, sweep, or impulse stimuli. WAV files must already match `--sample-rate`.

For music-like broadband validation, use seeded pink noise. Keep the seed in the command and metadata so the stimulus can be regenerated exactly:

```bash
cargo run -p pedalkernel-fx-loop -- capture \
    --device Scarlett \
    --send-channel 3 \
    --return-channel 3 \
    --signal pink \
    --duration 8 \
    --amp 0.12 \
    --seed 1592598566 \
    --out-dir hardware-goldens/ts9_noon \
    --label pink_seed_1592598566
```

Render a matching software reference from a `.pedal` file with its `.pedalhw` preset/mod metadata:

```bash
cargo run -p pedalkernel -- process \
    ../pedalkernel-pro/pedals/legends/screamer.pedal \
    hardware-goldens/ts9_noon/pink_seed_1592598566.stimulus.wav \
    /tmp/screamer_ts9_pink.wav \
    --pedalhw ../pedalkernel-pro/pedals/legends/screamer.pedalhw \
    --preset "Classic TS" \
    --mod "TS9 Conversion" \
    --no-calibrate
```

If `--pedalhw` is omitted, `process` looks for a sibling file with the same stem and the `.pedalhw` extension. Direct knob overrides still work after the flags and take precedence over preset values, for example `Tone=0.45`. Use `--no-calibrate` for hardware-reference comparisons when you want the raw circuit level without the `.pedal` file's product output normalization.

## Benchmarks

PedalKernel ships a Criterion benchmark harness. Run it on your machine to get authoritative per-pedal timing:

```bash
cargo bench --bench wdf_bench
```

Output lands under `target/criterion/`. The CI workflow also runs this on every push to `main` and uploads the report as a build artifact.
