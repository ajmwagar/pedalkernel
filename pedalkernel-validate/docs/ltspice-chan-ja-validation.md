# LTspice CHAN vs PedalKernel J-A Validation

PedalKernel keeps Jiles-Atherton as the runtime nonlinear transformer core. The
LTspice CHAN fixture in this directory is only an external behavioral reference:
it checks that the J-A transformer is in the same broad family of saturated,
hysteretic magnetic behavior without treating CHAN as the implementation target.

Run the LTspice fixture:

```sh
pedalkernel-validate/spice-circuits/magnetics/ja_vs_ltspice_chan_se.ltspice.cir
```

Export `V(out)` from LTspice as CSV, then import it:

```sh
cargo run -p pedalkernel-validate -- \
  --config pedalkernel-validate/ltspice-chan.yaml \
  --golden pedalkernel-validate/golden \
  import-csv-golden exported.csv \
  --suite magnetics_external \
  --test ja_vs_ltspice_chan_se \
  --signal sine \
  --column 'V(out)' \
  --duration 0.1
```

The importer resamples LTspice's variable-step `time,V(out)` export onto the
validation grid (`sample_rate * oversample`). Use `--no-resample` only for CSV
data that is already sample-aligned with the PedalKernel test signal.

Run the opt-in external suite:

```sh
cargo run -p pedalkernel-validate -- \
  --config pedalkernel-validate/ltspice-chan.yaml \
  --circuits pedalkernel-validate/circuits \
  --golden pedalkernel-validate/golden \
  run --suite magnetics_external
```

The tolerances are deliberately loose because CHAN and J-A are different
hysteresis models. Tightening this suite should wait until we have measured
transformer data or a vetted J-A SPICE reference.
