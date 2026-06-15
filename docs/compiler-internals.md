---
title: "Compiler internals"
description: "SPQR decomposition, stage routing, and the compiler passes that turn a .pedal file into a runnable processor."
section: "Internals"
weight: 85
source_commit: "222212fe33f0aa223f7d545d890a16654c17cca8"
watches:
  - pedalkernel/src/compiler/mod.rs
  - pedalkernel/src/compiler/compile.rs
  - pedalkernel/src/compiler/spqr.rs
  - pedalkernel/src/compiler/spqr_build.rs
  - pedalkernel/src/compiler/signal_flow.rs
  - pedalkernel/src/compiler/stage.rs
  - pedalkernel/src/compiler/compiled.rs
  - pedalkernel/src/compiler/rigid/
  - pedalkernel/src/compiler/blockwise.rs
  - pedalkernel/src/compiler/k_method.rs
  - pedalkernel/src/compiler/bias_analysis.rs
  - pedalkernel/src/compiler/topology.rs
  - pedalkernel/src/compiler/opamp_analysis.rs
  - pedalkernel/src/compiler/dsp_block.rs
  - pedalkernel-rt/src/routing.rs
  - pedalkernel-rt/src/route.rs
  - pedalkernel-rt/src/boundary_math.rs
---

# Compiler Internals

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
                         topology::classify_topologies
                          (Pass 1.5: per-op-amp context)
                                           │
                                           ▼
                         opamp_analysis::analyze_opamps
                          (Pass 2: feedback loop extraction)
                                           │
                                           ▼
                          dsp_block: behavioural gaps split,
                          generator nodes seeded
                                           │
                                           ▼
                                 Signal-flow groups
                                  (active barriers)
                                           │
                                           ▼
                          bias_analysis::classify_group_bias
                          (StaticBias vs SignalPath)
                                           │
                                           ▼
                         blockwise::try_build_blockwise
                          (NL cascades → chained blocks)
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
                        │                              │     ┌─ k_method::generate_k_table
                        │                              ├─────┤   (optional KTable fast path)
                        └─────────────┬────────────────┘     └─ (memoryless NL roots only)
                                      ▼
                          Stage vectors + StageRef
                          topological order
                                      │
                                      ▼
                            DspBlock::bind_runtime
                          (delay, BBD, VCO, VCA, spring)
                                      │
                                      ▼
                          routing::StageRoutePlan
                          (incident-wave coupling
                           between sub-networks)
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

## Pass 1.5: topology classification

Some routing decisions need a richer view of a component's neighbourhood than the bare graph gives. `compiler::topology` is a Pass 1.5 between graph construction and signal-flow grouping. It builds a `ResolvedTopology` from the `PedalDef` — union-find pin resolution plus per-component adjacency tables — and hands every component a `TopologyContext` so it can self-classify.

The motivating case is op-amps. A `Component::classify_topology()` override looks at its neighbours and returns one of `UnityGain`, `Inverting`, `NonInverting`, `BridgedTResonator`, `Allpass`, `Sallen-Key`, etc. Over time, each new topology migrates into the component's `classify_topology()` impl, shrinking the monolithic detector that used to live in `graph::find_opamp_feedback_loops()`.

## Pass 2: op-amp feedback analysis

`compiler::opamp_analysis::analyze_opamps()` is the merge step: it takes the Pass 1.5 self-classifications from each op-amp and reconciles them against the central feedback detector in `graph::find_opamp_feedback_loops()`. The output is a per-op-amp `OpAmpFeedbackInfo` plus a `DiodePairedOpAmp` shortlist for the Tube Screamer family (op-amps that share an NL junction with a diode clipper).

`build_opamp_feedback_stages()` then emits `WdfStage`s with the correct `OpAmpRoot` and `FeedbackConfig`. This is the place where `Rf` and `Ri` are extracted off the graph, the gain becomes `Rf/Ri` (inverting) or `1 + Rf/Ri` (non-inverting), and the feedback-pot pathway is set up so a runtime knob move recomputes the closed-loop gain in place.

Op-amp non-idealities (GBW, slew, rail saturation) attach to whatever stage ends up holding the op-amp output — they are not part of the scattering matrix and not part of this pass. See [NonIdealFxState](#nonidealfxstate) below.

## DSP block pre-pass

A small registry of high-level circuit elements — delay line / tap, BBD, VCO, VCA, spring reverb — never goes through SPQR. The `DspBlock` trait (see [DSP blocks](./dsp-blocks.md) for the full design) is consulted three times during compile:

1. **Terminal seeding.** Every block instance's input and output nodes are pushed into the SPQR terminal list, so the surrounding flow groups end cleanly at the block boundary rather than trying to reduce through it.
2. **Generator output injection.** Zero-input blocks (a `vco()` with no audio inputs) contribute their output nodes as signal sources — downstream stages don't need an upstream WDF stage to feed them.
3. **Behavioural-gap splitting.** A flow group that crosses a block boundary is split via union-find on the group's edges, with the block's ports as fixed cut points.

After every WDF / multi-NL / op-amp stage has been built, `bind_runtime_all()` walks the registry one more time and calls each block's `bind_runtime()` to attach its runtime body (`DelayLineBinding`, `VcoBinding`, …) to the `CompiledPedal`. A final `reject_unlowered_behavioral()` pass refuses to ship a `CompiledPedal` whose high-level components didn't end up with a binding — that's the "never ship silent" gate.

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
2. **Single nonlinear element**, coupled to passives. Before defaulting to a plain WDF tree, four upgrade checks run in order:
   - `try_varimu_3port` — promotes a vari-mu triode to a 3-port group with grid + plate exposed.
   - `try_bjt_two_port` — promotes a BJT to a 2-port group with base + collector exposed.
   - `try_linearized_ota` — promotes an envelope-controlled OTA into a coupled multi-NL solve.
   - `try_common_cathode_triode` — routes a common-cathode 12AX7-style stage (triode + cathode network + plate load + input coupling, typically a 6-edge group) to a `MultiNlStage` with the cathode and plate exposed as ports. Pre-2026-06 the check only matched 1-edge groups, so common-cathode triodes fell through to a plain `WdfStage` and the cathode-bypass cap interacted incorrectly with the plate resistor.
   If any of those fire, the group becomes a `MultiNlStage` with a coupled device group instead of a `WdfStage`. Otherwise it falls through to `plan_single_nl` and the single-NL WDF path.
3. **Parallel active branches at a convergence node.** When two active sub-trees (a triode plate and a JFET drain summing into the same coupling cap, for instance) share an output node, `plan_stages` sums them as parallel current sources into the convergence node rather than treating each as an isolated stage that races for ownership of the cap. This is the F13b rule and it landed the 808 snare convergence behaviour.
4. **All-passive** — falls to a feedforward `WdfStage` build, then either keeps wave-domain semantics (passive RootKind) or, for a linear rigid block, ends up wrapped inside a `MultiNlStage` with the `iir` or `state_space` field populated for fast evaluation. A non-SP passive R-node that would otherwise reduce to zero stages now routes to `Rigid` / MNA instead of being silently dropped.
5. **Unclaimed op-amps** (bridged-T, multi-path) trigger a nullor fallback that absorbs the op-amp into a synthetic `MultiNlPlan` with VCVS stamping into the MNA.

The coupling-aware legality checks from the previous section gate paths 4 and 5: if a subgraph would otherwise lower to a linear IIR/state-space form but `has_nl_reactive_coupling()` is true, it falls back to a heavier solver.

There used to be a "pot divider special case" path. It is gone. Pots are passive edges that participate in the normal SPQR reduction; the three-terminal split (`__aw` / `__wb`) is a control-binding concern, documented in the [controls and pots](./controls-and-pots.md) page.

## Blockwise decomposition

Some circuits — the TB-303 diode-transistor ladder is the canonical example — chain several nonlinear devices together with passive coupling networks between them. Treating that as one monolithic R-type adaptor would put every junction into one Newton-Raphson solve; per-iteration cost grows cubically in the number of NL ports, and convergence degrades when far-apart junctions interact through stiff coupling.

`compiler::blockwise` (introduced in commit `f732eaa`) sits between SPQR decomposition and stage building. It walks the SPQR tree, finds places where two or more NL components live in separate sub-trees connected only by linear/reactive passives, and emits a `BlockwisePlan`:

```rust
pub struct Block {
    pub nl_edges: Vec<usize>,
    pub linear_edges: Vec<usize>,
    pub reactive_edges: Vec<usize>,
    pub port_nodes: Vec<NodeId>,
}
pub struct BlockwisePlan {
    pub blocks: Vec<Block>,
    pub coupling_edges: Vec<usize>,
    pub port_nodes: Vec<NodeId>,
}
```

Each `Block` gets its own SPQR sub-decomposition, becomes one (or a few) WDF stages with a single nonlinear root, and connects to its neighbours through the passive `coupling_edges` (also lowered to WDF stages). The validation check requires at least two blocks, every block carries some reactive state of its own, and the coupling network contains no NL edges — otherwise the plan is rejected and `spqr_build` falls through to the monolithic R-type path.

The walker recognises two structural patterns: P-nodes whose children are NL Q-leaves from the same component (diode-connected BJTs, where base and collector share a net), and NL Q-leaf siblings inside S/R nodes (common-emitter BJTs, where B-E and C-E share the emitter node but have distinct base/collector endpoints). Boundary nodes shared by multiple blocks are resolved by inspecting the device's signal terminals — the block whose "common pin" (emitter / source / cathode) sits at the shared node owns the ground-side edges hanging off it. The fallback is to pick the lower-indexed block (upstream in cascade).

The payoff is that each individual block becomes a small single-NL WDF tree, eligible for [K-method tabulation](#k-method-tables) and for the per-stage NR warm-starting that single-root stages already do well. A 4-BJT ladder that the old pipeline ran as a 4×4 NR system is now four cascaded single-port solves with three coupling RC stages between them.

## K-method tables

For memoryless nonlinear roots — diodes, BJTs, JFETs, triodes — the per-sample Newton-Raphson solve can be replaced by a precomputed lookup table. The reflected wave `a_root` is sampled across the operating domain at compile time, and the runtime does a bilinear interpolation instead of iterating.

The table lives on the `WdfStage` itself:

```rust
pub struct KTable {
    pub dims: usize,              // 1 (1D) or 2 (2D)
    pub b_min: f64, pub b_max: f64,
    pub ctrl_min: f64, pub ctrl_max: f64,
    pub steps: usize,             // 1024 for 1D, 256 for 2D
    pub entries: Vec<f64>,
}
```

1D tables (1024 entries) cover single-junction devices: diodes, diode pairs, zeners. The single axis is `b_tree`, the incident wave. 2D tables (256 × 256 = 65 536 entries) cover BJTs / JFETs / MOSFETs / triodes — `b_tree` on one axis, the device's control voltage (`Vbe`, `Vgs`, `Vgk`) on the other. Pentodes report themselves as 3D K-method candidates but 3D tables are not yet generated.

Eligibility is gated by a `k_method_candidacy()` method on both the `Component` trait (`(bool, usize, &'static str)` — eligible, dims, rejection reason) and on `RootKind` itself (`(bool, usize)`). The two checks are mirrored — one is consulted at compile time when deciding whether to sample, the other at runtime when deciding whether to dispatch. Devices missing from either match arm fall through to NR.

Generation runs once per stage at compile time:

```rust
for ic in 0..steps {
    let ctrl = ctrl_min + tc * (ctrl_max - ctrl_min);
    for ib in 0..steps {
        let b = b_min + tb * (b_max - b_min);
        root.reset_nr_state();                  // cold-start per entry
        root.set_control_voltage(ctrl, 1.0, 0.0);
        let a = root.process(b, rp);            // NR solve
        entries.push(if a.is_finite() { a } else { 0.0 });
    }
}
```

Each grid point is a cold-start NR solve — `reset_nr_state()` zeroes the warm-start cache so the table value is a pure function of `(b_tree, ctrl)`, independent of sweep order. The control axis stores the AC signal portion, not the absolute voltage; the DC bias is subtracted at lookup so the same table works regardless of the stage's operating point.

The runtime dispatch in `WdfStage::process` is a single branch:

```rust
let a_root = if let Some(ref table) = k_table {
    if table.dims == 1 {
        table.lookup_1d(b_tree)
    } else {
        let ctrl = match root {
            RootKind::Bjt(b) => b.vbe() - b.vbe_bias(),
            RootKind::Triode(t) => t.vgk() - t.vgk_bias(),
            RootKind::Jfet(j) => j.vgs(),
            RootKind::Mosfet(m) => m.vgs(),
            _ => 0.0,
        };
        table.lookup_2d(b_tree, ctrl)
    }
} else {
    root.process(b_tree, rp)                    // NR fallback
};
```

A stage either has a `k_table: Option<KTable>` (lookup) or doesn't (NR). The two paths are mathematically equivalent in the limit of infinite table resolution; in practice the table trades floating-point work for L1 cache bandwidth and removes per-sample iteration variance — useful for tight CPU budgets and for circuits where NR convergence is borderline.

Stateful devices — BBDs, photocouplers, envelope-modulated OTAs, vari-mu triodes — are explicitly **not** K-method candidates because their I-V curve depends on history (charge buffer, thermal lag, envelope-shifted bias). See [nonlinear elements](./nonlinear-elements.md) for the complete per-device classification.

## Per-instance bias analysis

Before commit `5f782e1`, every triode used the same hardcoded `TRIODE_GRID_BIAS = -2.0`, every pentode used `PENTODE_GRID_BIAS = -8.0`, and BJTs ran at a fixed `0.6 V` Vbe. That worked well enough for circuits whose operating point happened to land near those constants — and badly for everything else.

`compiler::bias_analysis::classify_group_bias` extracts the real DC operating point from the circuit graph. It first classifies each signal-flow group:

```rust
pub(super) enum GroupBiasKind {
    SignalPath,
    StaticBias { dc_voltages: HashMap<NodeId, f64> },
}
```

A group is `StaticBias` when every edge in it has at least one rail terminal — i.e. the whole subgraph is `rail ↔ interior ↔ rail` resistor dividers with no edges spanning two non-rail nodes and no nonlinear elements. That's the topological signature of a bias network. A group that fails either condition carries audio and is on the `SignalPath`.

For bias groups, the DC voltage at each interior junction is computed by stamping the resistor conductances into a `G·V = I` system (caps are open-circuit at DC, so they're skipped) and solving via Gaussian elimination with partial pivoting. The systems are small (typically `n ≤ 5`).

Those DC voltages then flow into the NL roots through per-instance bias fields:

| Device | Field | Setter |
|---|---|---|
| `TriodeRoot` | `vgk_bias: f64` | `set_bias(vgk)` |
| `PentodeRoot` | `vg1k_bias: f64` | `set_bias(vg1k)` |
| `BjtRoot` | `vbe_bias: f64` | `set_bias(vbe)` |

At runtime, `set_control_voltage` adds the bias to the AC signal:

```rust
RootKind::Triode(t)  => t.set_vgk(t.vgk_bias() + input * compensation),
RootKind::Bjt(b)     => b.set_vbe(b.vbe_bias() + input * compensation),
RootKind::Pentode(p) => p.set_vg1k(p.vg1k_bias() + input * compensation),
```

The same machinery feeds the K-method generator: when a 2D table is sampled, the control axis runs across `ctrl_signal = vbe - vbe_bias` rather than absolute `Vbe`, so the table is shared across stages that have different bias points. At lookup, the bias is subtracted back out (see the K-method dispatch snippet above).

The old hardcoded constants are gone from the live source — they survive only as constructor-level fallbacks (`-2.0` for `TriodeRoot::new()`, `-8.0` for `PentodeRoot::new()`, `0.0` for `BjtRoot::new()`) used if `set_bias` is never called, which is rare in practice. Any tube or BJT stage built through the normal compile pipeline gets its operating point from the circuit graph.

## Cross-network coupling

A faithful LA-2A — or any rack unit that runs an input transformer, an internal gain cell, and an output transformer in series — is several galvanically isolated networks chained together. Inside one network, scattering and wave-domain math hold. Across a network boundary (a transformer primary → secondary, an optocoupler LED → photoresistor), the wave on one side has to drive a voltage on the other without sharing a node.

The runtime layer that does this lives in `pedalkernel-rt::routing` and `pedalkernel-rt::boundary_math`. It is **not** a fresh MNA stamp across the boundary — the implementation is wave-domain voltage injection that reuses the ideal-voltage-source reflection rule a stage already runs internally:

```rust
pub fn ideal_voltage_source_reflection(self, incident: IncidentWave) -> ReflectedWave {
    ReflectedWave(2.0 * self.0 - incident.0)
}
```

The donor stage publishes a `PortVoltage` at one of its boundary bindings. The compiler resolves the matching coupling port on the recipient stage by walking the blockwise (`BKM`) coupling-element adjacency and emits a `BkmBoundaryDrive`. Per sample, `push_differential_voltage_drives_for_ports` converts the donor's `PortVoltage` into a signed `IncidentWave` offset and sums it into the recipient port's incident vector *before* the WDF scattering matrix multiply runs. No matrix re-derivation, no extra solve.

`boundary_math.rs` carries the type-safety scaffolding for this: phantom-typed `DomainIndex<D>` distinguishes graph node IDs from MNA node IDs from scattering port IDs from processor port IDs, so the runtime cannot accidentally use a graph-side ID where a scattering port is expected. The conversion shapes (`PortVoltage`, `IncidentWave`, `ReflectedWave`, `PortOrientation`) are wrapper newtypes whose interconversions encode the orientation rules instead of letting hand-rolled `2V - a` math drift across sites.

`routing::StageRoutePlan` is the compile-time output: a flat list of `(stage_idx, vs_bindings, output_port_indices, boundary_drives)` indices the audio processor uses without rediscovering topology each sample. The plan is built once during `compile_via_spqr_with_options` and lives on the `CompiledPedal`.

## Diagnostic compile options

`CompileOptions` carries a handful of knobs for differential testing and offline reference renders. They default to the values that yield the best per-sample behaviour; flipping them produces a deliberately weaker compile so a regression can be isolated.

| Field | Default | Purpose |
|---|---|---|
| `oversampling` | `X4` | Oversampling factor for the runtime processor. |
| `tolerance` | `ideal()` | Component tolerance engine — Monte Carlo or worst-case. |
| `thermal` | `false` | Enable thermal drift on tubes / BJTs / JFETs. |
| `collapse_nl` | `false` | Force every NL element into one `MultiNlStage` (sidechain sub-circuits). |
| `skip_k_tables` | `false` | Skip [K-method](#k-method-tables) table generation — NR fallback at runtime. Main compile-time saver for debug builds. |
| `skip_blockwise` | `false` | Skip [blockwise decomposition](#blockwise-decomposition) and use a monolithic R-type adaptor. Available as `CompileOptions::force_monolithic()`. |
| `force_serial_blockwise` | `false` | Build blockwise rungs as serial WDF stages instead of a delay-free coupling stage. Intentionally breaks delay-free feedback for ladder-rung isolation. |
| `force_serial_blockwise_feedback_gain` | `0.0` | When `force_serial_blockwise` is on, wrap the serial chain in a one-sample feedback loop with this gain. |
| `disable_iir` | `false` | Don't synthesize IIR stages from rigid subgraphs; rigid passives fall through to state-space, active feedback stays in WDF. |
| `coupled_blockwise_newton` | `true` | Solve coupled blockwise K-method stages with full Newton/Jacobian iteration. Setting `false` uses table-driven fixed-point iteration — cheaper but less accurate near the operating-point boundary. |

The differential-testing pattern is to compile the same circuit twice — once with `CompileOptions::default()`, once with `force_monolithic()` — and compare sample-wise. Any deviation means blockwise is behaving as a dialect change rather than a pure optimisation, which is a bug.

The `compile_pedal_cached()` build.rs helper hashes all of these options into the cache key so a flag flip invalidates stale postcards.

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
- **`compiler::topology`** — Pass 1.5 per-op-amp neighbourhood classifier; backs `Component::classify_topology()`.
- **`compiler::opamp_analysis`** — Pass 2 feedback-loop extraction and `OpAmpRoot` stage emission.
- **`compiler::dsp_block`** — registry for delay / BBD / VCO / VCA / spring reverb; see [DSP blocks](./dsp-blocks.md).
- **`compiler::bias_analysis`** — `classify_group_bias()` and per-instance DC operating point extraction.
- **`compiler::blockwise`** — `analyze_blockwise()` / `try_build_blockwise()` for cascaded multi-NL circuits.
- **`compiler::k_method`** — `generate_k_table()` for 1D / 2D NL root tabulation.
- **`compiler::coupling`** — `port_semantic()`-driven legality predicates (`has_coupling_barrier`, `has_nl_reactive_coupling`, `device_parallels_passive`).
- **`compiler::rigid::mod`** — `classify_rigid()` and the lowering legality gates.
- **`pedalkernel-rt::routing` / `route` / `boundary_math`** — runtime cross-network coupling via wave-domain incident-wave injection.
- **`compiler::rigid::opamp_root`** — OpAmpRoot WDF element for multi-VCVS feedback.
- **`compiler::spqr_control`** — control binding and pot update propagation.
- **`pedalkernel-rt`** — runtime types: stages, `Stage` enum, `DynNode`, `LeafKind`, `WdfLeaf`, MNA, oversampling, loading, metering, thermal, nonlinear roots, and `crate::math` wrappers.

For the external API that wraps all of this, see the [Rust API](./api.md) page.
