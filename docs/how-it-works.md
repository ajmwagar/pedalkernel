# How PedalKernel Works

PedalKernel compiles your circuit description into a real-time Wave Digital Filter (WDF) engine. This document explains the compilation pipeline and how circuits become audio.

## The compilation pipeline

The compiler transforms your circuit into a WDF processing tree:

1. **Circuit graph** -- Union-Find merges connected pins into nodes. Components become edges.

2. **Nonlinear root identification** -- BFS from the input finds diodes, JFETs, triodes, pentodes, and BJTs. Each becomes a WDF root. Its neighboring passives form the WDF tree.

3. **Series-parallel decomposition** -- The passive subgraph around each nonlinear element reduces into a binary tree of Series and Parallel adaptors.

4. **Impedance balancing** -- A raw voltage source (Rp=1) in parallel with a 100k resistor would attenuate the signal to nothing. The compiler adjusts Vs port resistance to match its sibling, giving balanced gamma for efficient signal transfer.

5. **Per-sample processing** -- Four phases, zero allocation:
   - Scatter up: leaves to root (reflected waves propagate)
   - Root solve: Newton-Raphson on the nonlinear element (Shockley equation for diodes, Koren equation for triodes/pentodes)
   - Scatter down: root to leaves (incident waves propagate)
   - State update: capacitors and inductors latch

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

The tradeoff is CPU cost (still under 2% for complex pedals) vs accuracy (circuit-exact behavior).
