---
title: "Modeling limits"
description: "Where PedalKernel is pure WDF, where it approximates, and why."
section: "Internals"
weight: 90
source_commit: "222212fe33f0aa223f7d545d890a16654c17cca8"
watches:
  - pedalkernel/src/compiler/stage.rs
  - pedalkernel-rt/src/elements/nonlinear/
  - pedalkernel/src/compiler/rigid/
  - pedalkernel/src/loading.rs
  - pedalkernel/src/oversampling.rs
  - pedalkernel/src/thermal.rs
---

# Modeling Limits

PedalKernel is circuit-exact where it can be and pragmatic where it has to be. This page documents where the current implementation falls short of pure component modeling, so expectations are calibrated and contributors know where to push.

We keep the list honest rather than aspirational. Fixed items graduate off the list silently; they don't live here as a changelog.

## Pure WDF

These are compiled through the Wave Digital Filter tree and solved per-sample.

| Element | Model |
|---|---|
| Resistor, capacitor, inductor, potentiometer | Textbook one-port adaptors |
| Diode, diode pair, zener | WDF root, Wright Omega explicit solver (per-device Is/n, no iteration) |
| NPN / PNP BJT | `BjtRoot` (full Gummel-Poon: Early effect, high-injection knees, B-E/B-C leakage, junction capacitances, ohmic Rb/Re/Rc, transit times), Newton-Raphson on `(Vbe, Vce)`. Optional 2D K-method lookup table replaces NR when generated. |
| N- / P- JFET | WDF root, square-law |
| N- / P- MOSFET | WDF root, square-law |
| Triode, pentode, vari-mu | WDF root, Koren equation with softplus smoothing. Common-cathode triodes now route to a 2-port `MultiNlStage` (cathode + plate exposed) so the cathode-bypass cap interacts correctly with the plate load. |
| OTA (CA3080) | WDF root, hyperbolic-tangent transconductance |
| Transformers (audio, output, push-pull, center-tap) | Multi-port T-equivalent WDF model. The magnetizing-branch inductor can be replaced by a **Jiles-Atherton nonlinear core** (`DynNode::JaMagnetizingWithDcBias`) when the DSL config supplies `ja_ms` / `ja_a` / `ja_alpha` / `ja_k` / `ja_c`. Full B-H hysteresis: irreversible (`g`) plus reversible (`r`) magnetisation in the Chowdhury / Holters–Zölzer bulk susceptibility form, advanced by an implicit trapezoidal Newton solve (up to 12 iterations per sample). |
| Spring reverb tank | DspBlock (Välimäki/Parker 2010): up to 6 parallel dispersive lines, each a cascade of M=32 stretched first-order Schroeder allpasses inside a fractional-delay feedback loop with a one-pole damping LPF. Outputs summed through a 40 Hz DC blocker. Models `type4`, `type8`, `type9`, and Roland `re201_tank`. |
| Delay line / tap, BBD, VCO, VCA | DspBlock-routed; see [DSP blocks](./dsp-blocks.md). |

Per-device parameters — Shockley `Is`/`n` for diodes, Koren parameters for tubes, full SPICE Gummel-Poon parameter sets for BJTs (from `pedalkernel/src/model_lookup.rs`), thermal coefficients for BJTs / JFETs / tubes — are sourced from datasheets where possible and from industry-standard empirical fits otherwise.

**DC operating points are derived from the circuit.** Every triode, pentode, and BJT carries its own bias field (`vgk_bias`, `vg1k_bias`, `vbe_bias`) populated at compile time by `compiler::bias_analysis::classify_group_bias`. The bias analyser identifies static-bias subgraphs (rail ↔ interior ↔ rail resistor dividers) and solves their DC voltages via nodal analysis. The earlier global constants — `TRIODE_GRID_BIAS = -2.0 V`, `PENTODE_GRID_BIAS = -8.0 V`, `BJT_BASE_BIAS = 0.6 V` — are gone, surviving only as constructor fallbacks if `set_bias` is never called.

**Memoryless nonlinear roots can run from a precomputed K-method table** instead of Newton-Raphson. See the [nonlinear elements](./nonlinear-elements.md) catalogue and the [compiler internals K-method section](./compiler-internals.md#k-method-tables) for details. The two paths produce the same result up to interpolation error; the table is enabled when the device is K-method-eligible and the runtime decides to use the fast path.

## Approximations

These are modeled with a physical story but simplified for real-time cost or for modeling uncertainty.

**Op-amp feedback topology** is now extracted from the circuit graph. The compiler builds an `OpAmpRoot` inside a WDF tree, reading the feedback resistor `Rf` and the input resistor `Ri` off the graph; gain emerges from `Rf / Ri` for inverting topologies and `1 + Rf / Ri` for non-inverting, recomputed on each pot change. For circuits where the op-amp doesn't fit a clean WDF root — bridged-T filters, multi-VCVS feedback, ring loops — a nullor pass absorbs the op-amp into a multi-NL R-type stage, stamping the VCVS into the MNA. What is still approximate: closed-loop dynamics are handled by a post-processing layer rather than as part of the scattering matrix — see the next note.

**Op-amp non-idealities** (gain-bandwidth product, slew rate, rail saturation) are applied as a shared post-processor on top of every stage type, not as WDF elements. This means GBW-induced phase shift, slew-induced HF compression, and rail clipping all sit downstream of the scattering matrix. For most pedal circuits this is inaudible, but for precision topologies (servo loops, active filters where GBW is a design variable) it is a simplification worth knowing about.

**Rail saturation** is device-aware but not circuit-exact. Op-amps use a symmetric quintic-knee clipper with per-type output swing. BJTs use asymmetric saturation (hard positive knee, soft negative cutoff). FETs use a square-law pinch-off shape. Tubes use grid-conduction / plate-cutoff asymmetry.

**BBD delay** is a first-order physical model: per-stage charge leakage as a delay-dependent low-pass, NE571-style companding with mismatched attack/release, and clock feedthrough as a sine injection at the clock frequency. Real BBD nonlinearities (stage-to-stage variation, temperature drift, clock-to-signal intermodulation) are not captured.

**Multi-network rack circuits** (LA-2A-style chains of input transformer → gain cell → output transformer) couple between galvanically isolated sub-networks using wave-domain incident-wave injection (`pedalkernel-rt::routing::BkmBoundaryDrive`), not a fresh MNA stamp across the boundary. The donor stage publishes a `PortVoltage` at its boundary binding; the recipient port's incident wave is offset by the signed `2V − a` reflection rule before its scattering matrix multiply. This preserves per-sample cost but means the coupling is unidirectional within a sample — back-action from the recipient to the donor lands one sample late, the same one-sample delay every blockwise coupling carries. For most audio circuits that is inaudible; for ultra-low-feedback servo loops it is the next refinement.

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
