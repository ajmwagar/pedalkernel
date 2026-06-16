---
title: "Dynamics Dashboard"
description: "Compressor / limiter dynamics — WDF engine vs ngspice golden references, measured by the trusted dynamics tools. Honest gaps included."
type: "dynamics"
---

This page shows how the **PedalKernel WDF engine** reproduces the **dynamic behavior** of compressors and limiters — static gain curves, gain reduction, and attack/release ballistics — against its **ngspice golden references**. Every panel is generated from committed golden `.npy` files by [`tools/dashboard/generate_dynamics.py`](https://github.com/ajmwagar/pedalkernel/blob/main/tools/dashboard/generate_dynamics.py), driven by the `dynamics` subcommand of `pedalkernel-validate`, which measures both the WDF and ngspice goldens with the **same trusted dynamics primitives** that gate the test suite ([`pedalkernel::analysis`](https://github.com/ajmwagar/pedalkernel/blob/main/pedalkernel/src/analysis.rs): `compression_ratio`, `gain_reduction_db`, `measure_attack_seconds`, `measure_release_seconds`).

> This is a **mid-campaign snapshot**, not a victory lap. The dynamics path is one of the engine's known weak spots, and these graphs are built to show exactly where and by how much it diverges from SPICE.

**What dynamics validation is.** A compressor or limiter is defined by *how its gain changes with level and time*. We feed two deterministic stimuli through both the WDF engine and the ngspice reference:

- **Level sweep** — a 1 kHz tone stepped through a series of input levels. The steady-state RMS at each level gives a **static gain curve** (input dBVU vs output dBVU); its slope is the **compression ratio**, and the change in applied gain across the sweep is the **total gain reduction**.
- **Tone burst** — a single on/off burst. The output envelope across the burst traces the **gain-reduction-vs-time** transient, shown qualitatively: a non-compressing path stays flat, while a compressor's gain reduction builds over the burst. (Scalar attack/release times in ms are *not* reported here — a single-burst envelope-settle measurement reflects audio-level settling rather than true GR ballistics, and isn't reliable on the current reference.)

For all of these, **closer to ngspice is better**: where the orange WDF trace tracks the cyan ngspice trace, the engine reproduces the circuit's dynamics; where they diverge, the gap is real.

**A known gap, shown honestly.** Today only the **FET Leveler (1176-style) compressor** (`compressor/fet_leveler`) has dynamic-stimulus goldens. The WDF engine currently produces **~0 dB GR** because its **feedback compressor path is broken** (the envelope-detector → gain-element wiring does not close the loop — audit gap G5 family), so the static-curve panel shows a flat, linear WDF transfer (constant gain, no compression). That divergence is the point: a measured gap, not a hidden pass. The page auto-includes any future circuit that gains a `level_sweep` or `tone_burst` golden.

> **⚠ Preliminary reference.** Both sides of these graphs are early. The validation config *targets* ngspice ~4.2 dB GR (attack 5–10 ms, release 1–2 s), but the **committed `fet_leveler` ngspice golden does not yet match that** — it reads as a heavy, non-ideal attenuator (output near the noise floor at low input, large level-step transients, and an anomalous point near −5 dBVU). So treat the **absolute numbers — ratio, total GR, attack/release ms — as provisional**, and read the panels qualitatively: WDF applies constant gain (no dynamics) while the reference is level-dependent. Hardening the `fet_leveler` golden (or adding a cleaner compressor reference) is tracked follow-up work.
