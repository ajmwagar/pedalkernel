---
title: "How it works"
description: "The WDF compilation pipeline — from .pedal file to per-sample processing."
section: "Guide"
weight: 10
source_commit: "ba0372ed07318273d8d1a016ca9a572acc0a27df"
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

4. **DC bias analysis** — Each group is classified as a `StaticBias` network (only resistor dividers between rails) or a `SignalPath` (carries audio). Static-bias groups are solved at compile time by nodal analysis and their results land in the triodes, pentodes, and BJTs as per-instance `vgk_bias` / `vg1k_bias` / `vbe_bias` fields. The audio signal modulates around that real operating point rather than around a hardcoded global constant.

5. **Blockwise decomposition** — When several nonlinear devices are chained together through passive coupling (a TB-303 ladder, a cascaded triode stage), the compiler splits them into individual blocks. Each block becomes its own small WDF tree with one nonlinear root; the passives between them become coupling stages. This turns one large multi-port Newton-Raphson into several small single-port solves — each of which is then eligible for the K-method fast path described below.

6. **Per-group decomposition** — Each signal-flow group (or each block, after blockwise splitting) is routed by `compiler::plan::plan_stages`:
   - **Multiple nonlinear elements** in one rigid group become a single multi-NL stage that solves them all together via Newton-Raphson on an R-type adaptor.
   - **One nonlinear element** triggers three upgrade checks first (vari-mu 3-port, BJT 2-port, envelope-controlled OTA) — any of which promote it into a coupled multi-NL stage. Otherwise it lands in a plain WDF tree built by series/parallel reduction.
   - **All-passive** groups become WDF trees too; if the rigid residual is linear, it can carry a precomputed biquad cascade (IIR fast path) or state-space matrix as an internal optimisation.
   - **Unclaimed op-amps** (bridged-T, multi-path) fall through to a nullor pass that absorbs them into a multi-NL R-type stage with VCVS stamping.

7. **K-method tabulation** — For each WDF stage whose root is a memoryless nonlinearity (diode, zener, BJT, JFET, MOSFET, triode), the compiler samples the device's I-V curve across the operating domain into a `KTable` — 1024 entries for 1D devices, 256 × 256 for 2D. At runtime the stage interpolates from the table instead of running Newton-Raphson. The two paths are equivalent in the limit of infinite resolution; the table trades floating-point work for L1 bandwidth and removes per-sample iteration variance. Stateful devices (BBDs, photocouplers, envelope-modulated OTAs, vari-mu triodes) are not candidates — their I-V depends on history. See [nonlinear elements](./nonlinear-elements.md) for the per-device list.

8. **Stage assembly** — stages land in a few separate vectors: ordinary WDF stages, multi-NL stages, push-pull pairs (Fairchild-style differential triodes), and bare op-amp stages. A `stage_order` index threads the WDF and multi-NL stages into topological order; push-pull and op-amp stages process on their own schedule.

9. **Control binding** — Pot controls declared in the `.pedal` file are bound to the stages that contain them. When the user moves a pot at runtime, only the affected stages recompute.

## Per-sample processing

A compiled pedal processes one sample at a time by walking the stage order and dispatching to the right vector. Each stage kind runs its own code path:

- **WDF stages** run the classic four phases: scatter up from leaves to root, root solve on the nonlinear element (Wright Omega explicit solver for diodes and zeners, Newton-Raphson on the Koren equation for tubes, Newton-Raphson on the full Gummel-Poon model for BJTs, square-law for FETs), scatter down, then state update on capacitors and inductors. If the stage carries a K-method table, the root solve is replaced by a bilinear table lookup. Some WDF stages have purely-passive roots (`Passthrough`, `CapacitorRoot`, `PassiveRType`) and skip the root solve.
- **Multi-NL stages** run a multi-port Newton-Raphson over the coupled nonlinear devices on the R-type adaptor. If their `iir` or `state_space` field is set (linear-only passive children), the runtime takes that fast path instead.
- **Push-pull stages** evaluate two coupled triode halves as one block.
- **Op-amp stages** run an `OpAmpRoot` solve directly without a surrounding WDF tree.

Finally, every op-amp output passes through a shared **non-ideality post-processor** — a gain-bandwidth low-pass, a slew-rate limiter, and device-aware rail saturation. This is where op-amp character (TL072 vs. LM308 vs. JRC4558) mostly lives.

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
