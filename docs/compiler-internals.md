---
title: "Compiler internals"
description: "SPQR decomposition, stage routing, and the compiler passes that turn a .pedal file into a runnable processor."
section: "Internals"
weight: 85
source_commit: "ce2eb772992a4fc8a078aa43395c961b5ffc7907"
preview: true
watches:
  - pedalkernel/src/compiler/mod.rs
  - pedalkernel/src/compiler/compile.rs
  - pedalkernel/src/compiler/spqr.rs
  - pedalkernel/src/compiler/spqr_build.rs
  - pedalkernel/src/compiler/signal_flow.rs
  - pedalkernel/src/compiler/stage.rs
  - pedalkernel/src/compiler/compiled.rs
  - pedalkernel/src/compiler/rigid/
---

# Compiler Internals

> **Preview.** This page describes the SPQR compiler architecture being developed on the `feature/spqr-tree` branch. Most of what is covered here — signal-flow grouping, the `port_semantic`-driven coupling barriers, the `pedalkernel-rt` extraction, the multi-NL `iir` / `state_space` fast-path fields — is not yet on `main`. The overall shape is stable; specific type names and file paths may still move before the branch merges. Published here ahead of the merge so contributors can read along with the implementation.

This page is for contributors. It documents the architecture of the compiler that lives under `pedalkernel/src/compiler/` — the data structures, the decomposition algorithm, and the routing decisions that send each part of a circuit to the right solver.

If you only want the user-level story, the [how it works](./how-it-works.md) page covers it.

## The big idea

Every guitar circuit is a graph of components. Solving audio through it in real time means turning that graph into something a computer can evaluate per sample. There are two classic approaches:

- **Matrix solvers** (MNA, state-space) treat the whole circuit as one system of equations. General but slow — each sample is an `O(n³)` factorization or an `O(n²)` multiply.
- **Wave digital filters** trade generality for speed. A series/parallel tree of 2-port adaptors evaluates in `O(n)` per sample, and nonlinearities live at the tree's root where a scalar equation solves (Wright Omega for diodes and zeners — explicit, no iteration — Newton-Raphson for tubes and BJTs).

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
                        ┌──── plan_stages routing ────────┐
                        │                                  │
                Single-NL upgrade                 All-passive
                or multi-NL group                       │
                        │                              ▼
                        ▼                        SPQR tree
                  MultiNlStage                  (S / P / Q / R)
                  (R-type adaptor,                    │
                   optional iir /                     ▼
                   state_space fields)            WdfStage
                        │                              │
                        └─────────────┬────────────────┘
                                      ▼
                          Stage vectors + StageRef
                          topological order
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

Stages live in four parallel vectors on `CompiledPedal`, not a single tagged enum:

```rust
stages: Vec<WdfStage>,             // wave-domain stages, the workhorse
multi_nl_stages: Vec<MultiNlStage>,// rigid linear or coupled-nonlinear blocks
opamp_stages: Vec<OpAmpStage>,     // bare OpAmpRoot stages (rare)
push_pull_stages: Vec<PushPullStage>, // differential triode pairs
stage_order: Vec<StageRef>,        // topological traversal of WDF + MultiNL
```

`StageRef` itself is a slim two-variant enum (`Wdf(usize)` / `MultiNl(usize)`) that indexes into the first two vecs. Push-pull and bare op-amp stages are processed outside that ordering — push-pull tubes after the WDF passes, op-amp stages on a different schedule. There is **no** top-level `Stage` enum and no single `Vec<Stage>`.

What each kind covers:

- **`WdfStage`** is the workhorse. Carries an SPQR tree, a `RootKind` at the root, and the output extraction. The `RootKind` enum has roughly twenty variants, grouped into:
  - **Diode family** — `DiodePair`, `SingleDiode`, plus their explicit (Wright Omega) twins `ExplicitDiodePair` and `ExplicitSingleDiode`. New diode stages compile to the explicit form by default.
  - **Tubes** — `Triode`, `Pentode`, `VariMu`. Newton-Raphson on the Koren equation.
  - **Semiconductors** — `Jfet`, `JfetVr` (LFO-modulated source-follower for variable-resistor mode), `Mosfet`, `Ota`, `OpAmp`.
  - **Zener** — `Zener` for breakdown clamping.
  - **Passive-only roots** — `Passthrough` (`b = a`), `ShortCircuit` (`a = -b`), `VoltageSourceDriver`, `CapacitorRoot`, `InductorRoot`, `ResistiveTermination`. Used when an SPQR tree has no nonlinearity but the compiler wants the wave-domain semantics anyway (proper port termination, correct VS injection).
  - **`PassiveRType`** — an MNA-derived R-type adaptor for passive networks that aren't series-parallel reducible.
- **`MultiNlStage`** is a rigid block: an R-type adaptor with a multi-port Newton-Raphson solver wrapped around it. Holds a vector of single-port nonlinear devices (`NlDeviceKind`) and optionally a coupled `NlDeviceGroupKind` (3-port triode/pentode, 2-port BJT, Ebers-Moll BJT). Two important escape hatches live as fields on the same struct: `iir: Option<IirData>` and `state_space: Option<StateSpaceData>`. When the passive children of the R-type are linear-only, the compiler precomputes a biquad cascade (`iir`) for series-parallel-reducible passives, or a state-space matrix (`state_space`) for rigid passives. The runtime picks the cheapest available path; IIR wins over state-space when both are populated.
- **`OpAmpStage`** wraps a single `OpAmpRoot` for the rare case where an op-amp doesn't fit any of the other paths.
- **`PushPullStage`** processes Fairchild-style differential triode pairs as one unit so the two halves stay phase-coherent.

A previous `BlackFeedbackStage` for single-VCVS feedback groups was prototyped and removed. The op-amp feedback path is now uniformly an `OpAmpRoot` inside a `WdfStage`.

## Stage selection

Stage routing happens in `compiler::plan::plan_stages` (not in a `classify_rigid()` function — that name is gone). The decision tree, in order, for each signal-flow group:

1. **Multiple nonlinear elements** in one rigid group → `MultiNlPlan`. The R-type adaptor stamps every nonlinearity as an exposed port, and the runtime solves them all together.
2. **Single nonlinear element**, coupled to passives. Before defaulting to a plain WDF tree, three upgrade checks run in order:
   - `try_varimu_3port` — promotes a vari-mu triode to a 3-port group with grid + plate exposed.
   - `try_bjt_two_port` — promotes a BJT to a 2-port group with base + collector exposed.
   - `try_linearized_ota` — promotes an envelope-controlled OTA into a coupled multi-NL solve.
   If any of those fire, the group becomes a `MultiNlStage` with a coupled device group instead of a `WdfStage`. Otherwise it falls through to `plan_single_nl` and the single-NL WDF path.
3. **All-passive** — falls to a feedforward `WdfStage` build, then either keeps wave-domain semantics (passive RootKind) or, for a linear rigid block, ends up wrapped inside a `MultiNlStage` with the `iir` or `state_space` field populated for fast evaluation.
4. **Unclaimed op-amps** (bridged-T, multi-path) trigger a nullor fallback that absorbs the op-amp into a synthetic `MultiNlPlan` with VCVS stamping into the MNA.

The coupling-aware legality checks from the previous section gate paths 3 and 4: if a subgraph would otherwise lower to a linear IIR/state-space form but `has_nl_reactive_coupling()` is true, it falls back to a heavier solver.

There used to be a "pot divider special case" path. It is gone. Pots are passive edges that participate in the normal SPQR reduction; the three-terminal split (`__aw` / `__wb`) is a control-binding concern, documented in the [controls and pots](./controls-and-pots.md) page.

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

Because this is stage-type-independent, it attaches to any stage whose final value is an op-amp output — `OpAmpRoot`-using `WdfStage`s, multi-NL stages with op-amp ports, dedicated `OpAmpStage`s.

## Pot binding

Pots are graph edges whose resistance the user modulates at runtime. The compiler's `spqr_control` module walks every stage at compile time and records which stages hold which pots. When the user adjusts a control:

1. `set_control("Drive", 0.7)` dispatches to every bound pot.
2. Each pot's `set_pot(name, position)` updates the component resistance.
3. Stages containing that pot get `notify_pot_changed()`.
4. For a feedback-path pot, `notify_pot_changed()` reads the new resistance and calls `OpAmpRoot::set_feedback_pot_r()`.
5. The gain is recomputed. `NonIdealFxState::update_gain()` recalculates the GBW coefficient so the bandwidth tracks the new closed-loop gain.
6. `recompute_all()` propagates the update through WDF port resistances if the pot lives inside a WDF tree.

The goal is zero allocation per control change. Pots can be swept at audio rate without glitching.

## Coupling-aware optimisation

The compiler doesn't just decide *which* stage variant a subgraph becomes — it also decides whether a particular optimisation is *legal*. The classic foot-gun: lowering a subgraph to a linear IIR biquad is great for performance, but if a nonlinear element shares a node with a capacitor in that same subgraph, the biquad model can't represent the time-domain coupling between them. You'd get a working pedal that sounded subtly wrong.

The legality check is driven by `port_semantic()` on each component (see [the Component trait](./component-trait.md#port-semantics-and-coupling-aware-optimisation)) and a small set of predicates in `compiler/coupling.rs`:

- `has_coupling_barrier(graph, edges)` — any edge in the subgraph is `Nonlinear` or `ControlledConductance`. Generic optimisation barrier.
- `has_reactive_state(graph, edges)` — any edge is `Reactive` (capacitor, inductor).
- `has_nl_reactive_coupling(graph, edges)` — a `Nonlinear` edge shares a circuit node with a `Reactive` edge. **The decisive test for IIR legality.**
- `device_parallels_passive(comp_id, edge, edges, graph)` — a nonlinear or controlled-conductance device sits in parallel with a passive element. Used by passive-extraction edge cases.

`has_nl_reactive_coupling` deliberately excludes `ControlledConductance` from the barrier set: pots can be lowered to IIR because pot recompute updates biquad coefficients on demand. Only nonlinear I(V) curves create coupling that IIR fundamentally cannot model.

Two lowering paths in `classify_rigid()` consult this predicate before committing:

1. The single-VCVS linear path (reactive feedback to ground with resistive divider) — IIR is the cheap home, but only if no nonlinearity touches the reactive node.
2. The all-linear-passive path that goes straight to a biquad — same gate.

If either fires, the subgraph falls back to `General` (rigid linear or multi-NL solver) and stays heavier but correct.

This generalised system replaced a set of topology-specific exceptions. The clearest example: the old `bjt_collector_emitter_parallels_resistor()` check was a hard-coded BJT-shunt detector wired into one passive-extraction edge case. Its replacement, `device_parallels_passive()`, doesn't know what a BJT is — it asks `port_semantic()` whether the device is `Nonlinear` or `ControlledConductance`, then checks the graph topology. Adding a new device family doesn't require touching the optimiser; the trait override carries the information through.

## The pedalkernel-rt crate

The runtime — everything you need to *evaluate* a compiled pedal at audio rate — lives in a sibling crate, `pedalkernel-rt`, that compiles `no_std + alloc`. The split is:

- **`pedalkernel`** keeps the DSL parser, the SPQR/coupling pipeline, the graph analysis, layout/KiCad/BOM exporters — anything that runs once at compile time.
- **`pedalkernel-rt`** keeps `WdfStage`, `IirStage`, the `Stage` enum, `DynNode`, `WdfLeaf` and the `LeafKind` enum, MNA, oversampling, loading, metering, thermal, the nonlinear roots, and `crate::math` wrappers.
- The main crate re-exports the runtime types it cares about (`pub use pedalkernel_rt::PedalProcessor;`) so consumers don't have to know about the split.

Two pieces are worth flagging because they look unfamiliar in code:

**`LeafKind` enum.** WDF leaves used to be `Box<dyn WdfLeaf>` — a heap-allocated trait object. `no_std` builds without an allocator can't ergonomically use trait objects (Box requires `alloc`; trait-object dispatch wants vtables that play badly with serde). The enum replacement names every concrete leaf type as a variant — `Resistor`, `Capacitor`, `Inductor`, `VoltageSource`, `Pot`, `Photocoupler`, `JfetVr`, `SwitchedResistor`, `LeakyCapacitor`, `UnitDelay` — and dispatches by `match`. Adding a new leaf type means adding an enum variant and arms in three or four match statements.

**`crate::math` wrappers.** `f64::sin`, `f64::exp`, etc. are inherent methods that only exist with `std`. The math module wraps each of them: on `std` it's `#[inline(always)] x.sin()`, on `no_std` it's `libm::sin(x)`. Code in `pedalkernel-rt` calls `crate::math::sin(x)` rather than `x.sin()`. There's also a `fast_math` module with LUT-based `exp` and `tanh` for the hot paths inside the nonlinear root solvers.

`HashMap` similarly migrated from `std::collections` to `hashbrown` so it works in both worlds.

## Incremental WDF recompute

Pot updates used to walk the entire WDF tree to recompute every adaptor coefficient. With dirty-flag tracking, recomputation only touches the path from a changed leaf up to its root, and skips entire subtrees that have no dynamic leaves at all.

Two new fields on `DynNode::Binary`:

- `dirty: bool` — does this node need its `gamma` / `b1` / `b2` re-derived?
- `has_dynamic: bool` — does any leaf below this node have variable impedance?

`compute_dynamic_flags()` walks the tree once at compile time, marking `has_dynamic` on every ancestor of every dynamic leaf (pot, photocoupler, JFET-VR, switched resistor). At runtime, `set_pot_dirty()` walks from the changed leaf upward and flips `dirty` on each ancestor. `recompute_incremental()` then skips any node whose `has_dynamic` is false (purely static subtree) or whose `dirty` is false (already up-to-date).

For a typical tone stack with a single tone pot, this means roughly `O(log n)` work per pot change instead of `O(n)`. The full recompute path is preserved as a fallback for compile-time setup and for stages that haven't migrated yet.

`FeedbackConfig`, the older central type that held precomputed op-amp gain coefficients, is on its way out. The dynamic feedback divider is now read off the live tree via `notify_pot_changed()` + `recompute_incremental()`, and the type is deprecated rather than load-bearing.

## Current status

This branch is active development. The SPQR path is the only path — the old six-pass pipeline was deleted wholesale (6700 lines of tests with it).

What works: all 13 legend pedals compile successfully; the bulk produce correct audio; stage counts match expected values after signal-flow grouping; coupling-aware lowering gates correctly route bridged-T and similar topologies into multi-NL solvers. What is still red varies week to week — pot-sweep end-to-end tests, OpAmpRoot gain recomputation in particular non-inverting topologies, edge cases in the `iir` fast path inside multi-NL.

Numbers are moving week to week. The test suite and the commit log are the source of truth.

## Where to look

If you want to follow the code:

- **`compiler::spqr_build`** — top-level compiler entry point. `compile_via_spqr_with_options` is the function the rest of this page describes.
- **`compiler::spqr`** — SPQR tree construction and pendant extraction.
- **`compiler::signal_flow`** — directed BFS, OutputImpedance barriers, group assembly.
- **`compiler::coupling`** — `port_semantic()`-driven legality predicates (`has_coupling_barrier`, `has_nl_reactive_coupling`, `device_parallels_passive`).
- **`compiler::rigid::mod`** — `classify_rigid()` and the lowering legality gates.
- **`compiler::rigid::opamp_root`** — OpAmpRoot WDF element for multi-VCVS feedback.
- **`compiler::spqr_control`** — control binding and pot update propagation.
- **`pedalkernel-rt`** — runtime types: stages, `Stage` enum, `DynNode`, `LeafKind`, `WdfLeaf`, MNA, oversampling, loading, metering, thermal, nonlinear roots, and `crate::math` wrappers.

For the external API that wraps all of this, see the [Rust API](./api.md) page.
