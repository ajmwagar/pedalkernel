# LTspice CHAN vs PedalKernel J-A Validation

PedalKernel keeps Jiles-Atherton as the runtime nonlinear transformer core.
The LTspice CHAN fixture in this directory is only an external behavioral
sanity reference: it checks that the J-A transformer is in the same broad
family of saturated, hysteretic magnetic behavior without treating CHAN as the
implementation target.

This suite is named and tuned as `chan_behavioral_sanity_se` on purpose. Loose
THD tolerance here does not mean tight THD validation was dropped; it means CHAN
and J-A should not be treated as model-identical spectral references. Tight THD
coverage belongs against a model-identical J-A SPICE/reference implementation
and, for product work, against measured B-H loop fits from real iron.

The CHAN comparison is still useful for model-agnostic checks such as:

- Saturation onset drive level.
- THD-vs-frequency trend: distortion concentrated low and vanishing by a few
  hundred Hz.
- Cycle energy / loop area within a broad band.

Those checks catch geometry unit errors, Faraday scaling mistakes, sign flips,
and broken bias/settling behavior without pretending CHAN and J-A have identical
distortion spectra.

Run the LTspice fixture:

```sh
pedalkernel-validate/spice-circuits/magnetics/ja_vs_ltspice_chan_se.ltspice.cir
```

Run the fixture in batch mode if LTspice is available:

```sh
/Applications/LTspice.app/Contents/MacOS/LTspice -b \
  "$PWD/pedalkernel-validate/spice-circuits/magnetics/ja_vs_ltspice_chan_se.ltspice.cir"
```

Import the generated LTspice `.raw` file:

```sh
cargo run -p pedalkernel-validate -- \
  --config pedalkernel-validate/ltspice-chan.yaml \
  --golden pedalkernel-validate/golden \
  import-ltspice-raw-golden \
  pedalkernel-validate/spice-circuits/magnetics/ja_vs_ltspice_chan_se.ltspice.raw \
  --suite magnetics_external \
  --test chan_behavioral_sanity_se \
  --signal sine \
  --trace 'V(out)' \
  --duration 0.1
```

Alternatively, export `V(out)` from LTspice as CSV, then import it:

```sh
cargo run -p pedalkernel-validate -- \
  --config pedalkernel-validate/ltspice-chan.yaml \
  --golden pedalkernel-validate/golden \
  import-csv-golden exported.csv \
  --suite magnetics_external \
  --test chan_behavioral_sanity_se \
  --signal sine \
  --column 'V(out)' \
  --duration 0.1
```

The importers resample LTspice's variable-step trace onto the validation grid
(`sample_rate * oversample`). Use `--no-resample` only for data that is already
sample-aligned with the PedalKernel test signal.

Run the opt-in external suite:

```sh
cargo run -p pedalkernel-validate -- \
  --config pedalkernel-validate/ltspice-chan.yaml \
  --circuits pedalkernel-validate/circuits \
  --golden pedalkernel-validate/golden \
  run --suite magnetics_external
```

The amplitude tolerances are deliberately loose because CHAN and J-A are
different hysteresis models. The THD tolerance is deliberately very loose for
the same reason; it should not be tightened until the reference is a J-A
SPICE/reference implementation or measured transformer data fitted to J-A.
