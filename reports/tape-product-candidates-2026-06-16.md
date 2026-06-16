# Tape Product Candidates — Swiss / American Tube

Date: 2026-06-16

## Goal

Define two `.pedal` cores that can sit behind one professional 500-series-style
plugin UI:

- `Swiss Tape Channel`: clean solid-state tape-machine color path.
- `American Tube Tape Channel`: tube-front tape-machine color path.

The design target is not a literal clone. The first useful product step is a
stable, engine-friendly tape channel with recognizable studio lineage and a
shared control map suitable for VST3/Eurorack/500-series wrappers.

## Reference Lineage

Swiss reference:

- Public Studer/Revox B77 schematic set:
  https://www.reeltoreel.nl/studer/Public/Products/Revox/Revox_B77/
- Useful pages in `Revox_B77_Diagr.pdf`:
  - input amplifier `1.177.220`
  - record amplifier `1.177.230`
  - reproduce amplifier `1.177.250`

The B77 path is a solid-state machine: input amplifier, record-head drive,
head/repro loading and EQ, and line output buffering around a low-voltage rail.
The product core reduces this to a clean active record driver, a validated
voltage-driven Jiles-Atherton tape head, playback tilt, and output trim.

American tube reference:

- Ampex 601 archived schematic:
  https://web.archive.org/web/20060929035149/http://ftp.ampex.com/ampex/manuals/audio/601man/601schem.gif
- Ampex 601 archived manual:
  https://web.archive.org/web/20060929035309/http://ftp.ampex.com/ampex/manuals/audio/601man/601-man.pdf

The 601 schematic shows tube record/playback electronics and transformer-coupled
I/O. The product core keeps the practical part for this engine pass: a validated
12AX7 common-cathode front end feeding a stable active record driver and the
same shunt tape-head model as the Swiss path.

## Added Artifacts

Product `.pedal` cores:

- `pedalkernel-validate/circuits/product/swiss_tape_channel.pedal`
- `pedalkernel-validate/circuits/product/american_tube_tape_channel.pedal`

SPICE reference decks:

- `pedalkernel-validate/spice-circuits/product/swiss_tape_channel.spice`
- `pedalkernel-validate/spice-circuits/product/american_tube_tape_channel.spice`

WDF-only smoke/characterization gate:

- `pedalkernel-validate/tests/tape_product_candidates.rs`

The SPICE decks mirror default control settings and use the same behavioural
Jiles-Atherton shunt block as the existing `tape_head_saturation` reference.
The American deck uses the same Koren-style `12AX7` equations as the existing
tube validation decks.

## Validation Status

Command run:

```sh
cargo test -p pedalkernel-validate --test tape_product_candidates -- --nocapture
```

Result: `3 passed; 0 failed`.

Measured WDF smoke numbers at 48 kHz:

| Core | Default RMS | Default Peak | Drive Low RMS | Drive High RMS | Drive Low THD | Drive High THD |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Swiss Tape Channel | 0.011342 | 0.016150 | 0.010756 | 0.054096 | 0.00315 | 0.00228 |
| American Tube Tape Channel | 0.001696 | 0.003030 | 0.001074 | 0.008746 | 0.09597 | 0.09624 |

Interpretation:

- Swiss is already the stronger near-product core: healthy level, live Drive,
  stable active/head topology, and clean product positioning.
- American Tube now compiles and has a live Drive path, but it is intentionally
  a hybrid tube-front/active-record-driver core. Pure tube-drive variants tested
  during this pass were stable but far too low-level without an output
  transformer or unsafe high-gain op-amp makeup stage.
- The American path is useful as a second mode, but its exact gain staging needs
  calibration once ngspice goldens and rendered audio examples exist.

## Known-Good Basis

This pass deliberately avoids current engine weak spots:

- no compressor sidechain
- no multi-head delay/tape transport
- no spring reverb
- no FET gain cell
- no diode pedal clipping stack
- no switched tape-machine matrix

The cores build on known healthier pieces:

- `tape_head()` shunt topology from `circuits/tape/tape_head_saturation.pedal`
- common-cathode `12AX7` tube model from the tube validation suite
- simple active record-driver/op-amp topology that already appears in the rack
  tape example

## Next Validation Step

Ngspice was not available in this environment, so the SPICE decks have not yet
produced committed `.npy` goldens.

Once ngspice is installed, use the isolated product-candidate config:

```sh
cargo run -p pedalkernel-validate -- \
  --config pedalkernel-validate/config/product_tape_candidates.yaml \
  --skip-k-tables \
  --circuits pedalkernel-validate/circuits \
  --golden pedalkernel-validate/golden \
  generate-spice --suite product --spice-dir pedalkernel-validate/spice-circuits
```

The product suite is intentionally in a separate config for now. Adding it to
the default validation matrix before goldens exist would add missing-reference
noise. The next clean step is to generate ngspice and WDF goldens from this
config, then promote the suite/profile into the main matrix.

## Deferred Audio-Path Detail Pass

A later analysis should rank which real audio-path details are worth capturing
without a major engine lift. Early candidates are calibration level, playback HF
tilt, head loading, record/repro bias offset, simple LF head bump, transformer
I/O approximation, and per-mode output makeup. Larger lifts such as AC bias
oscillator, transport wow/flutter, print-through, full output transformer
saturation, and multi-head tape transport should stay outside the first product
core.
