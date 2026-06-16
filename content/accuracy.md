---
title: "Accuracy Dashboard"
description: "Honest, mid-campaign snapshot of WDF engine output vs ngspice golden references — every circuit, real dB errors, known gaps included."
type: "accuracy"
---

This page shows how closely the **PedalKernel WDF engine** reproduces its **ngspice golden references**, circuit by circuit. Every panel is generated from committed golden `.npy` files — the engine's wave-digital-filter output measured against SPICE ground truth — by [`tools/dashboard/generate_dashboard.py`](https://github.com/ajmwagar/pedalkernel/blob/main/tools/dashboard/generate_dashboard.py). For how the goldens themselves are produced (ngspice decks, the validation harness, and why goldens are never WDF-bootstrapped), see the [Validation](/book/validation/) chapter.

> This is a **mid-campaign snapshot**, not a victory lap. Failing circuits are shown, not hidden, and the headline numbers include accuracy gaps we are still working through. Where the engine diverges from SPICE, you will see exactly where and by how much.

**How to read the metrics.** Each circuit is scored on a handful of dB error figures comparing WDF output against the ngspice golden:

- **RMS** — normalized RMS error (dB) across the whole signal.
- **Peak** — peak error (dB), the worst-case sample deviation.
- **Spectral** — spectral error (dB), how the magnitude spectra differ.
- **THD** — THD error (dB), difference in total harmonic distortion (absent for some stimuli, shown as `—`).

For all of these, **lower is better**. Large negative numbers (e.g. **−200 dB**) mean the WDF output is numerically identical to ngspice; positive numbers are real, audible divergence. The per-circuit panels below plot **ngspice (cyan)** against **WDF (orange)** in the time domain with a difference trace, plus an FFT magnitude overlay so spectral discrepancies are visible at a glance.

**The dashed amber line on each FFT panel marks the audio Nyquist** (half the base sample rate). Circuits run at an oversampled rate, and PedalKernel then **anti-aliases** — band-limiting the ultrasonic content before it downsamples back to the audio band — so WDF output deliberately rolls off above this line. The ngspice golden, by contrast, is a raw transient sampled at the oversampled rate: it keeps the full nonlinear harmonics and aliasing up to its own (much higher) Nyquist. So a large spectral-error figure driven by content to the **right** of the line is the anti-aliasing doing its job, not an accuracy defect — it is the single biggest reason the nonlinear circuits show high `Spectral` dB while sounding correct.
