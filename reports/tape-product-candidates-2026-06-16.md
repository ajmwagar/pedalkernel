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
voltage-driven Jiles-Atherton tape head, engine-native bias/loading trim,
playback tilt, and output trim.

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
The `Bias` panel control is mirrored as a variable head-loading trim, not as a
true DC/RF bias source. The American deck uses the same Koren-style `12AX7`
equations as the existing tube validation decks.

## Validation Status

Command run:

```sh
cargo test -p pedalkernel-validate --test tape_product_candidates -- --nocapture
```

Result: `5 passed; 0 failed`.

Measured WDF smoke numbers at 48 kHz:

| Core | Default RMS | Default Peak | Drive Low RMS | Drive High RMS | Drive Low THD | Drive High THD |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Swiss Tape Channel | 0.010052 | 0.014302 | 0.009522 | 0.047883 | 0.00307 | 0.00219 |
| American Tube Tape Channel | 0.001611 | 0.002856 | 0.001008 | 0.008943 | 0.09616 | 0.09623 |

Control-surface binding and authority:

| Core | Runtime Bindings | Output Range | Tone Range at 8 kHz | Bias/Load Range |
| --- | --- | ---: | ---: | ---: |
| Swiss Tape Channel | Drive, Bias, Tone, Output | 16.03 dB | 3.84 dB | 1.46 dB level / 0.00023 THD delta |
| American Tube Tape Channel | Drive, Bias, Tone, Output | 16.03 dB | 1.54 dB | 1.48 dB level / 0.00005 THD delta |

Interpretation:

- Swiss is already the stronger near-product core: healthy level, live shared
  controls, stable active/head topology, and clean product positioning.
- American Tube now compiles and has the same live control surface, but it is
  intentionally a hybrid tube-front/active-record-driver core. Pure tube-drive
  variants tested during this pass were stable but far too low-level without an
  output transformer or unsafe high-gain op-amp makeup stage.
- The American path is useful as a second mode, but its exact gain staging needs
  calibration once ngspice goldens and rendered audio examples exist.
- The `Bias` control is product-useful now as a head-loading/saturation trim.
  It should not be marketed as a physically complete tape-bias oscillator.

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

## Product Gap Analysis

Current ready surface:

- Both cores parse, compile, pass signal, and expose the same four host-facing
  labels: `Drive`, `Bias`, `Tone`, `Output`.
- Runtime bindings are present for every control. `Drive` resolves through the
  black-feedback op-amp stage; `Bias`, `Tone`, and `Output` resolve in the
  passive/head/playback stage.
- The two cores can plausibly sit behind one modal plugin UI: Swiss as the clean
  solid-state tape mode, American as the lower-level tube-front color mode.

Transformer status:

- Linear transformer coupling should work for ratio/impedance color. The current
  transformer WDF leaf constructs a linear transformer/inductance path.
- Nonlinear transformer core saturation is not product-ready through
  `transformer()` today. The DSL carries Jiles-Atherton/core-looking fields, but
  the compiled transformer leaf does not consume them for magnetic saturation.
- For this product, transformer I/O can be added as a low-risk linear feature.
  Name-brand transformer saturation would be a separate engine/modeling pass, or
  it can be approximated with the already validated voltage-driven `tape_head()`
  saturation element.

Tape-bias status:

- A true DC/RF tape-bias source is not represented in these cores. The attempted
  DC rail injection compiled as a bound pot but produced no measurable audio
  authority in the WDF run.
- The current `Bias` panel is an engine-native proxy: it changes the head load
  around the tape-head shunt. That gives a stable, measurable control suitable
  for UI/product exploration while keeping the physics caveat explicit.

Calibration and validation gaps:

- Ngspice was not available here, so the product SPICE decks have not generated
  committed `.npy` goldens.
- American Tube is much quieter than Swiss at default settings and starts with
  high tube-front THD. It needs per-mode output makeup and listening/rendered
  audio calibration before wrapping.
- The product suite is isolated from the default validation matrix until goldens
  are generated and thresholds are assigned to a product/profile bucket.

Wrapper/UI gaps:

- The shared control labels are ready for host binding.
- The mode selector and VST3/500-series UI wrapper still need to be wired in the
  pro repo using the existing `single-pedal` prior art.
- A product-facing UI should treat `Bias` as a calibration/color control unless
  a later engine pass adds physical tape-bias modulation.

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
