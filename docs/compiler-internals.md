---
title: "Compiler internals"
description: "SPQR decomposition, stage routing, and the compiler passes that turn a .pedal file into a runnable processor."
section: "Internals"
weight: 85
---

# Compiler Internals

This page is for contributors. It documents the architecture of the compiler that lives under `pedalkernel/src/compiler/` — the data structures, the decomposition algorithm, and the routing decisions that send each part of a circuit to the right solver.

If you only want the user-level story, the [how it works](./how-it-works.md) page covers it.

## The big idea

Every guitar circuit is a graph of components. Solving audio through it in real time means turning that graph into something a computer can evaluate per sample. There are two classic approaches:

- **Matrix solvers** (MNA, state-space) treat the whole circuit as one system of equations. General but slow — each sample is an `O(n³)` factorization or an `O(n²)` multiply.
- **Wave digital filters** trade generality for speed. A series/parallel tree of 2-port adaptors evaluates in `O(n)` per sample, and nonlinearities live at the tree's root where a cheap Newton-Raphson solves a scalar equation.

Real pedal circuits are *mostly* series-parallel but not entirely. An op-amp with feedback, a differential pair, a bridged-T filter — these are irreducible by pure series/parallel reduction. The compiler's job is to find the biggest series-parallel subtrees it can, peel them off, and hand the remaining rigid pieces to a matrix solver.

This is exactly what an **SPQR decomposition** does in graph theory: it breaks a 2-connected graph into four kinds of subgraphs — Series, Parallel, Q (single edge), and Rigid — arranged in a tree. PedalKernel uses a custom variant tuned for electrical circuits rather than a stock reference algorithm. Entry point: `compiler::spqr_build::compile_via_spqr_with_options`.

## Pipeline overview

```
  .pedal DSL                      dsl::parse_pedal_file
      │                                    │
      ▼                                    ▼
   PedalDef  ── build ─────────▶  CircuitGraph
                                           │
                                           ▼
                                 Signal-flow groups
                                  (active barriers)
                                           │
                                           ▼
                        ┌──── Per-group decomposition ────┐
                        │                                  │
                Feedback group               Acyclic passive group
                        │                                  │
                        ▼                                  ▼
                  Rigid stage                     SPQR tree
                 (OpAmpRoot or                  (S / P / Q / R)
                  BlackFeedback)                       │
                        │                              ▼
                        │                        WDF / IIR / state-space
                        │                                 │
                        └─────────────┬───────────────────┘
                                      ▼
                                Vec<Stage>
                                (topological order)
                                      │
                                      ▼
                                CompiledPedal
                                      │
                           Control binding, ready to run
```

## CircuitGraph

The compiler's working data structure is a directed graph of nets and components. Nets become nodes; components become edges. Pin inference handles the cases where the DSL writer used the same net name twice or left an implicit connection.

Active elements (anything that behaves as a voltage source on its output — op-amps, unity-gain buffers, ideal voltage sources) expose their character through an `OutputImpedance` trait with two variants:

```rust
pub enum OutputImpedance {
    VoltageSource,  // zero output impedance — op-amp outputs, buffers
    Finite,         // everything else — passives, BJT collectors, diodes
}
```

This trait is what makes the next pass possible.

## Signal-flow grouping

The compiler does a directed BFS over the graph to find groups of passives that belong together. `VoltageSource` output nodes act as **barriers**: the BFS stops when it reaches an active element's output because the upstream circuit cannot load that node — impedance is effectively zero.

Concretely, each group is a maximal set of passive components reachable from a signal source without crossing an active output. The compiler used to do this with union-find over an undirected graph, which collapsed many unrelated passive networks together. Signal-flow grouping respects directionality and dramatically reduces the stage count:

| Pedal | Old stage count | New stage count |
|---|---|---|
| ProCo RAT | 16 | 5 |
| Tube Screamer | 19 | 5 |

Fewer stages means fewer WDF trees to maintain, faster compile, and cleaner cross-stage dependencies.

## SPQR decomposition

Each acyclic passive group is decomposed into an SPQR tree. The four node kinds:

- **S (Series)** — a chain of components connected end-to-end through degree-2 internal nodes. Compiles to a sequence of series adaptors.
- **P (Parallel)** — multiple components sharing the same two endpoints. Compiles to parallel adaptors.
- **Q (Quad / single edge)** — a single component. Either a passive WDF leaf or a nonlinear root.
- **R (Rigid)** — an irreducible subgraph with three or more boundary nodes. Not series-parallel reducible. Handed off to a matrix solver.

The decomposition algorithm (in `spqr.rs`) runs recursively:

1. Base cases — empty graph becomes R, single edge becomes Q.
2. **Pendant extraction** — repeatedly peel off dead-end edges, emitting each as a Q-leaf. See the next section; this is the trick that makes most real circuits reduce cleanly.
3. **Parallel detection** — if every remaining edge shares the same two endpoints, emit a P-node.
4. **Series detection** — find a degree-2 internal node, walk the chain it implies, and recursively decompose the remainder.
5. **Partial parallel** — mix of parallel branches with a rigid residual.
6. **R-node fallback** — anything that couldn't be reduced.

This is not a published reference algorithm (not Hopcroft-Tarjan, not Gutwenger-Mutzel). It is a purpose-built series/parallel detector tuned for the shapes that show up in guitar circuits.

## Pendant extraction

A **pendant** is an edge where one endpoint has degree 1 in the current subgraph. Extracting pendants before running series/parallel reduction is what makes this compiler work for real circuits.

Two non-obvious rules:

**Rail nodes always count as dead ends.** Ground, VCC, supply rails, and AC ground are treated as degree-1 endpoints even when many edges terminate on them. Otherwise a ground net with ten components attached would look like a degree-10 hub and mask the series structure of everything hanging off it. This rule lives in the pendant-detection loop and was the fix that made the T-junction canary tests pass.

**Extraction iterates.** After extracting a round of pendants, some internal nodes drop to degree 2, which reveals new series chains — which may in turn expose more pendants. The extractor runs in a loop until no more pendants exist.

Worked example — a T-junction of resistor, cap, and resistor around a degree-3 junction:

```
R1 ── junction ── R2 ── out
          │
          C1
          │
         gnd
```

After pendant extraction, `C1 → gnd` is peeled off as a Q-leaf (gnd is a rail dead-end). The junction now has degree 2. Series detection then sees `R1 — junction — R2` as a simple chain and reduces it. The decomposition lands cleanly where a naïve series-parallel detector would give up.

## Stage variants

Every stage the compiler emits is one of five kinds, stored in a single `Vec<Stage>`:

```rust
pub(super) enum Stage {
    Wdf(WdfStage),
    MultiNl(MultiNlStage),
    Iir(IirStage),
    StateSpace(StateSpaceStage),
    BlackFeedback(BlackFeedbackStage),
}
```

When each fires:

- **`Wdf`** — an SPQR tree that terminates in at least one nonlinear root (diode, JFET, MOSFET, tube, BJT, zener, OTA). Per-sample solver: scatter up → Newton-Raphson root solve → scatter down → state update.
- **`Iir`** — a purely linear series-parallel network. The compiler converts passive tone stacks, filters, and coupling networks into biquad or higher-order cascades when no nonlinearity is present. Much faster than running them as WDF trees.
- **`StateSpace`** — a linear rigid subgraph. `y = Cx + Du; x' = Ax + Bu`. Good for small matrix subcircuits like some high-order filters and multi-feedback networks.
- **`MultiNl`** — a rigid nonlinear subgraph where multiple nonlinear devices interact and cannot be factored into independent roots (e.g. a differential pair of BJTs with shared tail current). One Newton-Raphson over a small state vector.
- **`BlackFeedback`** — a single-VCVS acyclic feedback group (op-amp with passive feedback). Gain from Harold Black's formula, no iterative solve. Covered in detail below.

Stages are stored in topological order so processing just walks the vec. Helper accessors (`as_wdf`, `as_iir`, etc.) return `Option<&T>` for safe downcasting.

## BlackFeedbackStage

Most op-amp circuits in guitar pedals are single-amp with passive feedback. Solving them as full WDF trees is overkill — a closed-loop gain of `Rf / Ri` from Harold Black's feedback equation gives the right answer at a fraction of the cost.

`BlackFeedbackStage` compiles such groups directly:

```rust
pub(super) struct BlackFeedbackStage {
    rf: f64,                    // feedback resistance (can follow a pot)
    ri: f64,                    // input resistance (fixed)
    inverting: bool,
    fx_state: NonIdealFxState,  // GBW / slew / rails post-processor
    stored_gbw: f64,
    sample_rate: f64,
    signal_flow_distance: usize,
}
```

Runtime per sample:

1. If a pendant tree (input coupling network) hangs off the inverting input, run it as passive WDF to get the pre-gain voltage.
2. Multiply by the Black gain: `Rf / Ri` (inverting) or `1 + Rf / Ri` (non-inverting).
3. Pass the result through the non-ideality post-processor (next section).

The compiler extracts `Rf` and `Ri` by walking the feedback path in the circuit graph. For a non-inverting topology, `Rf` is any feedback edge not touching GND; `Ri` is any group edge touching GND with resistance. For inverting, the roles flip around the summing junction.

When the feedback path contains a pot (drive, distortion, sustain controls), moving the pot calls `set_feedback_pot_r()` which recomputes the gain and updates the GBW coefficient. See the "Pot binding" section below.

The fallback for circuits this stage can't handle — multi-VCVS, nested feedback, rigid irreducible subgraphs — is still `OpAmpRoot` embedded in a full WDF tree with an MNA adaptor at the rigid part.

## NonIdealFxState

Op-amp character — the reason a TL072 sounds different from an LM308 — mostly lives in the stage-independent post-processor:

```rust
pub(super) struct NonIdealFxState {
    gbw_coeff: f64,   // GBW-derived single-pole lowpass coefficient
    gbw_state: f64,
    max_dv: f64,      // slew-rate limit per sample
    prev_out: f64,
    v_max: f64,       // rail saturation voltage
}
```

Three operations in series, applied to every stage output:

1. **GBW lowpass** — a single-pole IIR whose cutoff is `GBW / gain`. As gain goes up, bandwidth goes down. An LM308 (GBW ≈ 1 MHz) at gain 100 has a cutoff around 10 kHz, which is audible.
2. **Slew rate limit** — clamps `|Δv|` per sample to the device's rated slew. LM308 is 0.4 V/µs; TL072 is 13 V/µs. Slew limiting at high drive levels is audible HF compression.
3. **Rail saturation** — a soft clipper at `v_max ≈ supply / 2 − 1.5 V`, shaped per device family. Op-amps use a symmetric quintic knee; BJT output stages use asymmetric saturation; etc.

Because this is stage-type-independent, it attaches to `BlackFeedbackStage`, `OpAmpRoot`-using `WdfStage`s, and anywhere else an op-amp output is the final value.

## Pot binding

Pots are graph edges whose resistance the user modulates at runtime. The compiler's `spqr_control` module walks every stage at compile time and records which stages hold which pots. When the user adjusts a control:

1. `set_control("Drive", 0.7)` dispatches to every bound pot.
2. Each pot's `set_pot(name, position)` updates the component resistance.
3. Stages containing that pot get `notify_pot_changed()`.
4. For a feedback-path pot, `notify_pot_changed()` reads the new resistance and calls `OpAmpRoot::set_feedback_pot_r()` (or the equivalent on `BlackFeedbackStage`).
5. The gain is recomputed. `NonIdealFxState::update_gain()` recalculates the GBW coefficient so the bandwidth tracks the new closed-loop gain.
6. `recompute_all()` propagates the update through WDF port resistances if the pot lives inside a WDF tree.

The goal is zero allocation per control change. Pots can be swept at audio rate without glitching.

## Current status

This branch is active development. The SPQR path is the only path — the old six-pass pipeline was deleted wholesale (6700 lines of tests with it).

What works: all 13 legend pedals compile successfully; 7 of 9 produce correct audio; stage counts match expected values after signal-flow grouping. What is still red: some pot-sweep end-to-end tests, a handful of BlackFeedbackStage unit tests around multi-stage coupling, and OpAmpRoot gain recomputation for certain non-inverting topologies.

Numbers are moving week to week. The test suite and the commit log are the source of truth.

## Where to look

If you want to follow the code:

- **`compiler::spqr_build`** — top-level compiler entry point. `compile_via_spqr_with_options` is the function the rest of this page describes.
- **`compiler::spqr`** — SPQR tree construction and pendant extraction.
- **`compiler::signal_flow`** — directed BFS, OutputImpedance barriers, group assembly.
- **`compiler::stage`** — stage variants including `BlackFeedbackStage` and `NonIdealFxState`.
- **`compiler::compiled`** — the `Stage` enum and helper accessors.
- **`compiler::rigid::opamp_root`** — OpAmpRoot WDF element for multi-VCVS feedback.
- **`compiler::spqr_control`** — control binding and pot update propagation.

For the external API that wraps all of this, see the [Rust API](./api.md) page.
