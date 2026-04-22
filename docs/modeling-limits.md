---
title: "Modeling limits"
description: "Where PedalKernel is pure WDF, where it approximates, and why."
section: "Internals"
weight: 90
---

# Modeling Limits

PedalKernel is circuit-exact where it can be and pragmatic where it has to be. This page documents where the current implementation falls short of pure component modeling, so expectations are calibrated and contributors know where to push.

We keep the list honest rather than aspirational. Fixed items graduate off the list silently; they don't live here as a changelog.

## Pure WDF

These are compiled through the Wave Digital Filter tree and solved per-sample.

| Element | Model |
|---|---|
| Resistor, capacitor, inductor, potentiometer | Textbook one-port adaptors |
| Diode, diode pair, zener | WDF root, Newton-Raphson Shockley (per-device Is/n) |
| N- / P- JFET | WDF root, square-law |
| N- / P- MOSFET | WDF root, square-law |
| Triode, pentode, vari-mu | WDF root, Koren equation with softplus smoothing |
| OTA (CA3080) | WDF root, hyperbolic-tangent transconductance |
| Transformers (audio, output, push-pull, center-tap) | Multi-port WDF element |

Per-device parameters — Shockley Is/n for diodes, Koren parameters for tubes, thermal coefficients for BJTs / JFETs / tubes — are sourced from datasheets where possible and from industry-standard empirical fits otherwise.

## Approximations

These are modeled with a physical story but simplified for real-time cost or for modeling uncertainty.

**Op-amps** are treated as per-device-type gain with rail saturation and slew-rate limiting. The feedback topology (inverting vs non-inverting, feedback-network impedance, virtual ground) is **not** modeled as part of the WDF tree. Op-amp character in PedalKernel therefore reflects slew-rate, output swing, and rail-saturation shape — but not feedback-dependent frequency response.

**BJT gain stages** contribute a per-device gain factor (2.5× for standard small-signal NPN/PNP, per-type for germanium variants) rather than participating in the WDF tree as a root element. CE / CB / CC topology, collector load, emitter degeneration, and bias-point shifts collapse into that scalar. JFETs and tubes are treated differently — they are full WDF roots.

**Rail saturation** is device-aware but not circuit-exact. Op-amps use a symmetric quintic-knee clipper with per-type output swing. BJTs use asymmetric saturation (hard positive knee, soft negative cutoff). FETs use a square-law pinch-off shape. Tubes use grid-conduction / plate-cutoff asymmetry.

**BBD delay** is a first-order physical model: per-stage charge leakage as a delay-dependent low-pass, NE571-style companding with mismatched attack/release, and clock feedthrough as a sine injection at the clock frequency. Real BBD nonlinearities (stage-to-stage variation, temperature drift, clock-to-signal intermodulation) are not captured.

**Interstage loading** between pedals is a first-order RC voltage divider. Real pedal-to-pedal interaction is frequency-dependent and slightly nonlinear — especially guitar pickup into a Fuzz Face.

**Tube parameters** are representative fits, not per-unit. A `triode(12ax7)` uses textbook Koren coefficients; individual tubes in the wild vary significantly. Adding per-unit variation via the tolerance engine is possible but not automatic.

## Numerical compromises

The solver is tuned for real-time safety rather than maximum precision.

- Newton-Raphson iterations cap at 16 per sample with a `|ΔV| < 1 µV` early-exit. Quadratic convergence handles typical Shockley and Koren loads in 4–8 iterations; convergence failure under extreme drive falls back to the last iterate.
- Exponential arguments clamp to `[-500, 500]` to avoid IEEE-754 overflow. At extreme voltages behaviour becomes piecewise-linear.
- Triode / pentode plate voltage clamps to `[-50 V, 500 V]` — a physically motivated range for guitar and audio circuits rather than an arbitrary one.
- Leakage conductance is a global `1e-12 S` (1 TΩ) — within the real-world `Icbo` range for semiconductors in cutoff, sufficient to keep Newton-Raphson from degenerating at zero-voltage crossings.
- Thermal state updates every 1000 audio samples (~21 ms at 48 kHz). Thermal time constants are 300–900 seconds, so this is three orders of magnitude faster than the dynamics warrant and costs nothing per sample.

## Not yet modeled

Real-world phenomena we would like to model but have not.

- Shared-supply sag across a pedalboard.
- Ground-loop and crosstalk between parallel signal paths.
- Speaker / cabinet physics (Thiele-Small, voice-coil compression, cone breakup, cabinet resonance).
- Oversampling latency reporting for DAW compensation.

See the [roadmap](./roadmap.md) for the items we plan to tackle next.
