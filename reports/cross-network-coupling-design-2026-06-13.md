# Cross-Network Coupling — A General Engine Facility

**Date:** 2026-06-13 · **Status:** design for review · **Branch:** la2a-1to1

## Thesis

The optocoupler (LED→LDR, optical) and the transformer (primary↔secondary,
magnetic) are *two instances of one idea*: a component whose electrical
terminals span **two galvanically-isolated networks**, coupled through a
**non-electrical domain**. Today the engine has no name for this. The
transformer is special-cased as a single multi-port WDF adaptor; the
optocoupler is faked as a scalar `set_led_drive` control with no electrical
LED port. This doc proposes the general mechanism and shows opto + transformer
as its first two clients. It does **not** rip out the transformer — see §5.

## 1. How transformers couple today (TIGHT / instantaneous)

`TransformerComp` (pedalkernel/src/compiler/components/transformer.rs) returns
`GraphRole::Transformer` (component.rs:389) and `StampResult::Skip`
(transformer.rs:75-84) — it is *not* MNA-stamped. Instead `make_leaf`
(transformer.rs:86-114) builds ONE runtime node: `DynNode::TransformerNode`
(dyn_node.rs:509), whose secondary winding is a **child** of the adaptor
(`Transformer { secondary, turns_ratio, rp, b_sec }`, dyn_node.rs:60-66). The
3-winding case uses `RTypeAdaptor::three_winding_transformer`
(transformer.rs:105). At runtime the adaptor scatters up
(`reflected_with_state`, dyn_node.rs:651-657: `b_sec = secondary.reflected…`,
then reflects with `turns_ratio`) and back down — **both windings resolved in a
single solve of one tree**. Mutual inductance couples them within one sample
via shared flux. The windings are galvanically isolated (no shared node) yet
electrically co-solved. This is *tight* coupling: the coupling quantity (flux)
has no perceptible memory at audio rate, so simultaneous solution is exact.

## 2. How the optocoupler is split today (DELAYED / stateful)

There is **no LED electrical port**. The LED side is a scalar control:
`Photocoupler::set_led_drive(0..1)` (controlled.rs:166-205) advances a two-rate
CdS cell state (`state_fast`/`state_slow`, asymmetric rise/fall taus,
controlled.rs:170-183) and writes `self.resistance`. The LDR side is a WDF leaf
implementing `WdfLeaf`/`ControlledResistance` (controlled.rs:282-321) — a
time-varying resistor living in the audio network. The two sides are in
**different networks** and **cannot be co-solved like transformer windings**:
the cell has *memory* (ms–s lag), so `R_ldr(t)` depends on **past** light, not
the instantaneous LED current. Binding is keyed on an `EnvelopeFollower`
element's `.out` net (bind.rs:716-755, `build_envelope_jfet_bindings`); a plain
circuit net into `.led` produces no binding — **GAP G** (la2a.pedal:50-58).

The engine *already* has a working delayed-coupling primitive: the
**sidechain**. `SidechainProcessor` (stage.rs:6620-6642) compiles a sub-circuit
into its own `CompiledPedal`, processes the tapped signal, and feeds the result
back through a **one-sample delay** `cv_delayed` (processor.rs:3811-3834: solve
network → read `sc.cv_delayed` → modulate → store new CV). This is the F5
feedback-tap convention (processor.rs:1016-1020, 3665-3668): feed-forward taps
act this sample; feedback taps take effect next sample. **That delay is the
discrete-time loop closure, and it is free.**

## 3. The two flavors — and why they differ

| | (A) TIGHT / instantaneous | (B) DELAYED / stateful |
|---|---|---|
| Physical coupling | mutual inductance (flux) | optical/EL (carrier lag) |
| Coupling memory | none at audio rate | ms–s (the whole point) |
| Solve | one multi-port element, both sides together | each side in its own network |
| Loop closure | within the single solve | one-sample delay |
| Cost | one bigger scattering matrix | two small solves + a scalar |
| Client | transformer | optocoupler (T4B), future relay |

The dividing line is **whether the coupling domain has audio-rate memory**. If
no (flux), simultaneous solution is correct and cheap-enough → keep it as a
multi-port WDF adaptor. If yes (light, heat, mechanical inertia), simultaneous
solution is *wrong* (it would ignore the lag) AND would force a joint matrix
across two otherwise-independent networks → use the delayed path.

## 4. The general abstraction: `CrossNetworkCoupler`

A component declares:

- **(a) electrical edges in possibly-different flow groups.** Each edge names
  a `coupling_side` id. The compiler must place a side's edges in their own
  network and **must not fuse the two sides into one** (§6).
- **(b) an internal coupling-domain state** — `flux`, `light`, future
  `temperature`, `armature_position`. One scalar (or small vector) owned by the
  component, persisted across samples.
- **(c) per-edge coupling rules**: each edge either **CONTRIBUTES** to the
  state (a *drive* edge: e.g. LED current → light) and/or **READS** from it (a
  *sense* edge: e.g. light → LDR resistance / port parameter). A transformer
  edge does both, instantaneously (flux ↔ both windings).
- **(d) loop-closure policy**: `Tight` (resolve within one solve) or
  `Delayed { samples: 1 }` (the F5 / sidechain convention) when the coupling
  forms a feedback path.

```text
trait CrossNetworkCoupler {
    fn sides(&self) -> &[CouplingSide];        // each = a (sub)set of edges + a network tag
    fn coupling_state(&self) -> CouplingDomain;// Flux | Light | Thermal | ...
    fn policy(&self) -> CouplingPolicy;        // Tight | Delayed { samples }
    // drive: given this side's solved port quantities, update state
    fn contribute(&mut self, side: SideId, port: PortSample, dt: f64);
    // sense: given state, set this side's port parameter for the next solve
    fn read(&self, side: SideId) -> PortParam; // e.g. R_ldr, or turns coupling
}
```

**Transformer expresses as flavor A**: one side per winding, `coupling_state =
Flux`, `policy = Tight`, `contribute`/`read` collapse into the existing single
adaptor scatter (no per-sample scalar hop needed — it *is* the multi-port
solve). **Optocoupler expresses as flavor B**: side `led` is a drive edge
(`contribute`: LED current → light, using the two-rate cell that already lives
in `Photocoupler`), side `ldr` is a sense edge (`read`: light → resistance),
`coupling_state = Light`, `policy = Delayed { 1 }`. The cell's `set_led_drive`
math (controlled.rs:166-205) becomes `contribute`; `port_resistance`
(controlled.rs:284) becomes `read`.

## 5. Does the transformer unify, or stay special-cased?

**Keep the transformer on the tight path; route the optocoupler through the new
delayed path.** Rationale: the multi-port WDF adaptor is correct and already
shipped; forcing it through a generic `contribute/read` scalar hop would (a)
add a one-sample delay that mutual inductance does not have, and (b) split one
exact solve into two coupled ones. The abstraction's value is that **both are
describable** by the same trait (sides + coupling-domain state + policy), so the
mechanism, validation, and DSL surface are uniform — but `policy() == Tight`
lowers to the existing `TransformerNode`/R-type adaptor, while
`policy() == Delayed` lowers to the new evaluator. This is the same pattern as
`DspBlock`, where one registry covers BBD/VCA/VCO/spring/delay with
type-specific guts (dsp_block.rs:144-152).

## 6. Compiler capability needed

A `CrossNetworkCoupler` whose edges land in two flow groups **without being
merged**. Today the graph would either share a node (galvanic — wrong) or, for
the LED-as-control case, drop the LED edge entirely. We need:

- **Per-side edges, no fusion.** The graph builder assigns each side its own
  network; `find_flow_groups` (signal_flow.rs:1220) must never union across the
  coupler. This is the *opposite* of `merge_shared_active_node_sccs`
  (signal_flow.rs:678) — the coupler is an explicit **non-merge barrier**.
- **Galvanic cut, like DspBlock.** `DspBlock` already cuts the graph at its
  audio pins (`StampResult::Skip` + `Behavioral` edge, dsp_block.rs:5-10) and
  each side compiles to separate serial stages bridged at runtime via
  `node_signals` reads/writes (dsp_block.rs:91-101). The delayed coupler reuses
  *exactly this gap-cut*, but the bridge is not an audio insert — it is a
  **coupling-state hop**: network 1 solves → `contribute` updates state →
  network 2's `read` sets a WDF-leaf parameter for next sample.
- **Runtime order:** solve net 1 → `contribute` → solve net 2 (which `read`s
  last sample's state). The delay is implicit in evaluation order + the F5
  convention; no extra matrix, no joint solve.

**Contrast with `DspBlock`:** a DspBlock *replaces a chunk of ONE network* with
a behavioral processor (it has audio in/out ports on the same signal path,
PortRole::Audio, dsp_block.rs:50-61). A cross-network coupler **does not carry
audio between its sides** — each side stays a genuine electrical port in its own
network; only a *non-electrical scalar* crosses, with memory. DspBlock = "cut
the wire, run DSP, rejoin the wire." Coupler = "two wires that never touch,
linked by a lagged physical quantity." Distinct mechanism; they can share the
gap-cut plumbing (`all_boundary_nodes`, dsp_block.rs:238) but not the bridge.

## 7. DSL surface

Minimal change, internal coupling preferred:

- The optocoupler's LED becomes a real **electrical port-pair** (`led.a`,
  `led.b`) wired into the sidechain network — *replacing* the
  `ModulationSinkKind::PhotocouplerLed` modulation-sink path. In la2a.pedal:237
  `EL_drive.b -> PC1.led` becomes `EL_drive.b -> PC1.led.a` /
  `… -> PC1.led.b` (LED across the EL-drive winding). The coupling
  (light) stays **internal to the one `photocoupler` component** — no user-
  visible coupling net.
- **No new "coupling-domain net" syntax** is warranted yet. Internal coupling
  suffices for opto and transformer (both are single components owning both
  sides). A `coupling_net` syntax would only matter if two *separate* DSL
  components had to share one domain (e.g. discrete LED + discrete LDR), which
  no current target needs. Defer.
- Transformer DSL is unchanged.

## 8. First client: the LA-2A T4B

The faithful T4B (la2a.pedal:126-237) is **flavor B end-to-end**: the rectified
sidechain network (out tap → R37 → 12AX7 → 6AQ5 → EL-drive winding) drives the
LED *electrically*; light (with the cell's lag) sets the audio-network LDR; the
1-sample delay closes the feedback loop. This directly resolves:

- **GAP G** (la2a.pedal:50-58, bind.rs:716-755): the LED gets a real port and
  `contribute` is fed from the solved EL-drive node, not from a phantom
  `envelope_follower`. The detector finally moves the cell from circuit state.
- **GAP H** (la2a.pedal:59-63, controlled.rs:85-94): once `contribute` owns the
  cell, the coupling-domain state is the natural home for **GR-history**
  (program-dependent τ) — add a slow accumulator that lengthens release after
  heavier/longer GR. (Out of scope to *fix* here; the abstraction makes it a
  state extension, not a new code path.)

GAP F (transformer step-down, ~19 dB, la2a.pedal:64-67, transformer.rs:86-113)
is **orthogonal** — it lives on the tight path and is not touched by this work.

## 9. Realtime contract

- Each side solved in its own network — **no extra matrix, no joint solve**
  (the delayed path is strictly cheaper than fusing two networks).
- Coupling state is a **cheap per-sample scalar update** (`contribute`/`read`),
  exactly the cost of today's `set_led_drive` (controlled.rs:166-205) plus a
  node read. The §3 matrix-cost rule still applies only to the *one* WDF-stamped
  parameter the `read` mutates (the LDR R), throttled like any controlled
  resistance (stage.rs:1565-1575).
- The delay is **free** — it is evaluation order, identical to
  `SidechainProcessor::cv_delayed` (processor.rs:3811-3834).

## 10. Phased plan

1. **Trait + policy enum.** Add `CrossNetworkCoupler` (sides, coupling domain,
   policy, contribute/read). Implement `policy() == Tight` as a thin wrapper
   that lowers to today's `TransformerNode` (no behavior change; proves the
   abstraction covers the shipped transformer).
2. **Delayed evaluator.** Reuse the DspBlock gap-cut + `node_signals` plumbing
   (dsp_block.rs) to place the two sides in separate stages; add the
   coupling-state hop (solve net1 → contribute → net2 reads last state).
3. **Optocoupler as client.** Give `photocoupler` a `led` electrical port;
   move `set_led_drive` math into `contribute`, `port_resistance` into `read`;
   delete the `PhotocouplerLed` modulation-sink binding path (bind.rs:716-755).
4. **LA-2A wiring + GAP G acceptance.** Re-point la2a.pedal:237 to the LED
   port; flip the gap-G `#[ignore]`/red acceptance test (la2a_acceptance.rs).
5. **GAP H (separate bead).** Add GR-history state to the coupling domain.

## Decisions (owner, 2026-06-13)

- **SUBSUME `SidechainProcessor`.** `CrossNetworkCoupler` is the more generic
  concept; a sidechain is a delayed coupler with `coupling_state = Cv`. Subsume
  it — its existing clients (Dyna Comp OTA sidechain, 670/push-pull) migrate
  onto the coupler with a **zero-behavior-change bar** (the DspBlock-
  generalization discipline: byte-identical on existing sidechain circuits
  first, then the optocoupler lands as the first new *optical* domain).
- **TIGHT path is already done; the entire new build is the DELAYED path.** The
  transformer already implements tight coupling (multi-port co-solve, §1) — the
  trait merely *describes* it, no new lowering. NOTE: GAP F (step-down scaling)
  is a *bug inside* that existing tight path, not a missing capability — fixed
  separately on `fix/transformer-step-down`.
- Net work breakdown (all delayed-path): (1) define `CrossNetworkCoupler`
  generalizing `SidechainProcessor`; (2) migrate sidechain clients,
  zero-behavior-change; (3) optocoupler as first optical client (GAP G + GAP H).

## Open questions (remaining)
- **Multi-sample delay** — is `Delayed { samples: 1 }` ever insufficient
  (e.g. a coupling domain with transport delay)? Probably not for opto/relay;
  keep `samples` configurable but default 1.
- **Where does `contribute` read its drive quantity** — port voltage, port
  current, or power? LED drive is current-ish; relay coil is current; EL panel
  is voltage. The `PortSample` shape (§4) must expose all three.
- **Validation:** should the compiler *reject* a galvanic short across a
  coupler's two sides (user accidentally shares a node), since that defeats the
  isolation the component exists to model?
- **Tight↔Delayed for the same device:** a leaky transformer at extreme HF has
  flux memory — do we ever want a transformer on the delayed path? Not now;
  note it as a future knob on `policy()`.
