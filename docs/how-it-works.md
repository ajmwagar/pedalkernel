---
title: "How it works"
description: "The WDF compilation pipeline — from .pedal file to per-sample processing."
section: "Guide"
weight: 10
source_commit: "95744ce1cdd9c2cdec3550bfdce9879b1737312c"
preview: true
watches:
  - pedalkernel/src/compiler/
  - pedalkernel/src/tree.rs
  - pedalkernel/src/dsl.rs
---

# How PedalKernel Works

PedalKernel compiles your circuit description into a real-time Wave Digital Filter (WDF) engine. This page is the guide-level story. The [compiler internals](./compiler-internals.md) page goes deeper on data structures, algorithms, and routing decisions.

## The compilation pipeline

The compiler transforms a `.pedal` file into a runnable processor through a single pass built around an **SPQR decomposition** of the circuit graph. SPQR stands for Series / Parallel / Q (single edge) / Rigid — the four kinds of subgraphs a 2-connected graph can be decomposed into. Each kind maps to a different solver strategy.

Roughly:

1. **DSL parse** — `dsl::parse_pedal_file` turns the `.pedal` source into a `PedalDef` AST: components, nets, controls, supplies.

2. **Circuit graph build** — Components become edges. Nets become nodes. Pin inference handles implicit connections. The result is a directed graph where signal flow is explicit.

3. **Signal-flow grouping** — Active elements whose outputs behave as voltage sources (op-amps, unity-gain buffers) act as *barriers* in the signal flow. Passives between barriers form one group; each group will become one or more stages. The barriers matter because on the output side of an op-amp, the upstream circuit can't load it — impedance is effectively zero — so there's no WDF benefit to treating the full passive network as one tree.

4. **Per-group SPQR decomposition** — Each signal-flow group is decomposed:
   - An **acyclic passive group** becomes a WDF tree via series/parallel reduction.
   - A **single-VCVS feedback group** (one op-amp with passive feedback) becomes a `BlackFeedbackStage` — gain computed from Harold Black's formula, `Rf / Ri`, extracted from the graph.
   - A **multi-VCVS or complex feedback group** becomes an `OpAmpRoot` embedded in a WDF tree, with an MNA adaptor handling the irreducible rigid part.
   - A **pot-divider group** is special-cased as `Parallel(R_aw, R_wb)` so pot positions map to resistances directly.
   - An **irreducible rigid subgraph** falls back to one of the matrix-based solvers: MNA, IIR (when linear), state-space, or multi-NL.

5. **Stage-list consolidation** — Every stage is pushed into a single `Vec<Stage>` in processing order. Five stage kinds live in that enum: `Wdf`, `MultiNl`, `Iir`, `StateSpace`, and `BlackFeedback`. The order is topological — downstream stages read the upstream ones.

6. **Control binding** — Pot controls declared in the `.pedal` file are bound to the stages that contain them. When the user moves a pot at runtime, only the affected stages recompute.

## Per-sample processing

A compiled pedal processes one sample at a time by walking the stage list in order. Each stage kind runs its own code path:

- **WDF stages** run the classic four phases: scatter up from leaves to root, root solve on the nonlinear element (Wright Omega explicit solver for diodes and zeners, Newton-Raphson for tubes and BJTs with the Koren and Ebers-Moll / Gummel-Poon equations, square-law for FETs), scatter down, then state update on capacitors and inductors.
- **BlackFeedback stages** apply a linear gain to their input — no iterative solver. If a pendant tree (input coupling network) is attached, that part is run as passive WDF first.
- **IIR stages** run biquad or higher-order filter cascades; the compiler converts series-parallel passive networks into biquads when no nonlinearity is present.
- **State-space stages** run a small `y = Cx + Du; x = Ax + Bu` update for rigid linear subcircuits.
- **MultiNL stages** run a Newton-Raphson iteration over a bundle of nonlinear devices whose interactions cannot be factored into separate roots.

Finally, every stage output passes through a shared **non-ideality post-processor** — a gain-bandwidth low-pass, a slew-rate limiter, and device-aware rail saturation. This is where op-amp character (TL072 vs. LM308 vs. JRC4558) mostly lives, once the feedback topology has already set the gain.

## Example: Big Muff topology

Two cascaded clipping cells. Signal-flow grouping puts each cell in its own group because there's a transistor gain stage between them. Within each group, SPQR finds a simple series/parallel structure around the diode pair:

```
Stage 1 (WDF):                    Stage 2 (WDF):
   [DiodePair D1]                    [DiodePair D2]
         |                                 |
   ParallelAdaptor                   ParallelAdaptor
    /          \                      /          \
  Series      R1(10k)              Series       R2(47k)
  /     \                          /     \
Vs    C1(100n)                   Vs    C2(100n)
```

Each stage independently solves the diode equation and feeds the result into the next. The passive coupling network between stages shows up as additional passive groups handled by IIR or WDF adaptors depending on whether it's linear.

## Supply voltage effects

Pedals run at 9V by default. Crank it to 12V or 18V for more headroom — the WDF engine and the non-ideality post-processor both respond. The clipping threshold shifts, gain stages swing further before saturating, and the op-amp rail-saturation voltage adjusts to the new supply. Just like plugging a real Tube Screamer into an 18V adapter.

For tube amps, supply voltage goes up to 500V. Set `supply 460V` in a `.pedal` file and the pentode plate curves, triode operating points, and Newton-Raphson bounds all adjust to match real B+ rails.

```rust
proc.set_supply_voltage(12.0);   // More headroom, cleaner clipping
proc.set_supply_voltage(18.0);   // Even more — like a Voodoo Lab Pedal Power
proc.set_supply_voltage(460.0);  // Fender Twin Reverb B+ rail
```

## WDF vs traditional modeling

Traditional pedal modeling uses measured impulse responses or simple math models ("soft clipper" = tanh). PedalKernel solves the circuit equations per-sample, capturing subtle interactions:

- A 220nF feedback cap clips differently than a 100nF (exact corner frequency affects feedback path impedance)
- Germanium diodes soften the edges; silicon clips harder
- A 12AX7 saturates like a 12AX7; there's no "generic triode" mode
- Changing supply voltage reshapes the entire nonlinear curve

The tradeoff is CPU cost vs accuracy. See the [performance page](./performance.md) for measured per-pedal budgets across the example library — most pedals land comfortably in the low single digits of CPU on a modern machine at 48 kHz.
