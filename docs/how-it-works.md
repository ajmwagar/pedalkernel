---
title: "How it works"
description: "The WDF compilation pipeline — from .pedal file to per-sample processing."
section: "Guide"
weight: 10
---

# How PedalKernel Works

PedalKernel compiles your circuit description into a real-time Wave Digital Filter (WDF) engine. This document explains the compilation pipeline and how circuits become audio.

## The compilation pipeline

The compiler lives in `pedalkernel::compiler` and transforms your circuit in roughly these stages:

1. **DSL parse** — `dsl::parse_pedal_file` turns a `.pedal` source file into a `PedalDef` AST (components, nets, controls, supplies).

2. **Graph build** — Union-Find merges connected pins into nodes; components become edges. Pin inference handles implicit connections.

3. **Topology analysis and stage planning** — The compiler identifies clipping stages, classifies nonlinear regions, and decomposes the circuit into WDF stages. Multi-stage pedals are split so each nonlinear root has its own local tree.

4. **Nonlinear root identification** — BFS from the input finds diodes, zeners, JFETs, MOSFETs, triodes, pentodes, and OTAs. Each becomes a WDF root; its neighbouring passives form the WDF tree around it.

5. **Series-parallel decomposition** — The passive subgraph around each nonlinear element reduces into a binary tree of Series and Parallel adaptors.

6. **Impedance balancing** — A raw voltage source (Rp = 1) in parallel with a 100 kΩ resistor would attenuate the signal to nothing. The compiler adjusts Vs port resistance to match its sibling, giving balanced scattering coefficients for efficient signal transfer.

7. **Active device modeling** — Op-amp analysis, BJT bias points, and gain-stage wiring are resolved. (See [modeling limits](./modeling-limits.md) for where these are WDF roots vs. where they are simplified.)

8. **Validation and codegen** — Voltage compatibility and component warnings fire at compile time. The final `CompiledPedal` is ready to run with zero allocation.

## Per-sample processing

Once compiled, processing one audio sample runs four phases:

- **Scatter up** — leaves to root (reflected waves propagate inward)
- **Root solve** — Newton-Raphson on the nonlinear element (Shockley equation for diodes, Koren for triodes and pentodes, square-law for FETs)
- **Scatter down** — root to leaves (incident waves propagate outward)
- **State update** — capacitors and inductors latch their new state

## Example: Big Muff topology

Two cascaded clipping stages. Each diode pair gets its own WDF tree:

```
Stage 1:                          Stage 2:
   [DiodePair D1]                    [DiodePair D2]
         |                                 |
   ParallelAdaptor                   ParallelAdaptor
    /          \                      /          \
  Series      R1(10k)              Series       R2(47k)
  /     \                          /     \
Vs    C1(100n)                   Vs    C2(100n)
```

Each stage independently solves the diode equation and feeds the result into the next stage. The passives around each nonlinear element are already reduced to the tree shown above.

## Supply voltage effects

Pedals run at 9V by default. Crank it to 12V or 18V for more headroom -- the WDF engine models how the active elements respond to higher rail voltage. The clipping threshold shifts, the gain stages swing further before saturating, the whole feel opens up. Just like plugging a real Tube Screamer into an 18V adapter.

For tube amps, supply voltage goes up to 500V. Set `supply 460V` in a `.pedal` file and the pentode plate curves, triode operating points, and Newton-Raphson bounds all adjust to match real B+ rails.

```rust
proc.set_supply_voltage(12.0);   // More headroom, cleaner clipping
proc.set_supply_voltage(18.0);   // Even more -- like a Voodoo Lab Pedal Power
proc.set_supply_voltage(460.0);  // Fender Twin Reverb B+ rail
```

## WDF vs traditional modeling

Traditional pedal modeling uses measured impulse responses or simple math models ("soft clipper" = tanh). PedalKernel solves the circuit equations per-sample, capturing subtle interactions:

- A 220nF feedback cap clips differently than a 100nF (exact corner frequency affects feedback path impedance)
- Germanium diodes soften the edges; silicon clips harder
- A 12AX7 saturates like a 12AX7; there's no "generic triode" mode
- Changing supply voltage reshapes the entire nonlinear curve

The tradeoff is CPU cost vs accuracy. See the [performance page](./performance.md) for measured per-pedal budgets across the example library — most pedals land comfortably in the low single digits of CPU on a modern machine at 48 kHz.
