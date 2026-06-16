# Dynamics goldens (WDF engine snapshots)

These `.npy` files are **WDF engine output snapshots** used to render the
[Dynamics dashboard](../../docs) without re-running the engine on every build.
Each `compressor/<circuit>/level_sweep.npy` is the per-level *settled measurement
window* of a 1 kHz level sweep (−40..0 dBVU, 5 dB steps), concatenated, at 48 kHz.
The `dynamics` subcommand segments them into static gain curves.

**These are NOT ngspice/accuracy goldens.** They are intentionally kept out of
`golden/` so WDF-bootstrapped *display* snapshots are never confused with the
ngspice-derived *validation* goldens (which gate pass/fail).

## Refreshing

They are frozen snapshots — they go **stale when the compressor engine changes**.
Regenerate (re-runs the engine, ~20 s) with:

```
cargo run --release -p pedalkernel-validate -- \
  --golden pedalkernel-validate/golden \
  --report-dynamics data/dynamics.json \
  dynamics --regen --timestamp "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
```

Then re-render the PNGs with `tools/dashboard/generate_dynamics.py` and commit.
