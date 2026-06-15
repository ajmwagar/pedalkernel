---
title: "DSP blocks"
description: "The DspBlock lowering pass — how delay/tap, BBD, VCO, VCA, and spring reverb get plugged into the SPQR pipeline."
section: "Internals"
weight: 87
source_commit: "222212fe33f0aa223f7d545d890a16654c17cca8"
watches:
  - pedalkernel/src/compiler/dsp_block.rs
  - pedalkernel/src/compiler/delay_lowering.rs
  - pedalkernel/src/compiler/bbd_lowering.rs
  - pedalkernel/src/compiler/vco_lowering.rs
  - pedalkernel/src/compiler/vca_lowering.rs
  - pedalkernel/src/compiler/spring_lowering.rs
  - pedalkernel-rt/src/oscillator.rs
  - pedalkernel-rt/src/envelope.rs
  - pedalkernel-rt/src/glide.rs
  - pedalkernel-rt/src/elements/nonlinear/spring.rs
---

# DSP Blocks

Some circuit elements are too high-level to compile into raw WDF adaptors. A delay line is a buffer with a tap; a VCO is a phase accumulator with a PolyBLEP shaper; a spring reverb is a Schroeder allpass cascade inside a dispersive feedback loop. None of those map cleanly to "scatter up, root solve, scatter down."

The **DspBlock** pattern is the lowering pass that takes these elements off the SPQR pipeline before it tries to reduce them. Each block kind owns its compile-time recognition, its runtime binding, and its place in the topology graph.

## The trait

`DspBlock` is a trait, registered through a static slice — there's no central match statement:

```rust
pub(super) trait DspBlock {
    fn handles(&self, type_tag: &str) -> bool;
    fn component_ids(&self, pedal: &PedalDef) -> Vec<String>;
    fn io(&self, pedal: &PedalDef, graph: &CircuitGraph)
         -> Vec<(String, BlockIo)>;
    fn bind_runtime(&self, pedal: &PedalDef,
                    compiled: &mut CompiledPedal, sr: f64)
                    -> Result<(), String>;
}
```

Each block kind is a unit struct (`DelayLineBlock`, `BbdBlock`, `VcoBlock`, `VcaBlock`, `SpringBlock`) implementing the trait. They live in `compiler/dsp_block.rs` and per-block files: `delay_lowering.rs`, `bbd_lowering.rs`, `vco_lowering.rs`, `vca_lowering.rs`, `spring_lowering.rs`.

`handles(type_tag)` is the recognition step — `DelayLineBlock::handles("delay line")` returns true. `component_ids()` lists every instance of that type in the pedal AST. `io()` returns each instance's typed port signature:

```rust
pub struct BlockIo {
    pub inputs:  Vec<BlockPort>,
    pub outputs: Vec<BlockPort>,
}
pub enum PortRole { Audio, Cv, Gate, Clock, Sync }
```

`bind_runtime()` is what actually wires the block into the compiled processor — it builds the runtime struct (a `DelayLineBinding`, a `VcoBinding`, etc.), assigns control targets, and walks any associated modulation graph (LFO → BBD clock, envelope follower → VCA CV).

## Pipeline integration

The DspBlock registry is consulted at three points during `compile_via_spqr_with_options`:

1. **Terminal seeding.** `all_boundary_nodes()` returns every input/output node of every block instance. SPQR uses those as terminals so a flow group ends cleanly at a block boundary rather than trying to reduce through the block.

2. **Generator output injection.** `all_generator_output_nodes()` returns nodes that are driven by zero-input blocks (a `vco()` with no audio inputs). The downstream signal-flow analysis treats these as source nodes — they don't need an upstream stage to feed them.

3. **Behavioral-gap splitting.** `split_groups_at_behavioral_gaps()` walks each flow group and, if a block's port nodes sit interior to the group, splits it. This is union-find over the group's edges with the block's ports as fixed cut points.

After SPQR has run and built the WDF / multi-NL / op-amp stages around the blocks, `bind_runtime_all()` calls `bind_runtime()` on every registered block. The runtime structs (`DelayLineBinding`, `VcoBinding`, `VcaBinding`, `SpringBinding`, `BbdBinding`) land on the `CompiledPedal` alongside the WDF stages and execute per sample inside `pedalkernel-rt::processor`.

A final pass — `reject_unlowered_behavioral()` — fails the compile if any component that requires a DspBlock to do its work didn't end up with a binding. The error message lists the offending component IDs. This is the "never ship silent" gate: a `vco()` with no wired output, or a `delay_line()` with an unbound tap, is a compile error, not a silent stage.

## The blocks

### Delay line and tap

`DelayLineBlock` handles `type_tag == "delay line"`. The I/O signature is one Audio input, one main Audio output, plus one extra Audio output per `tap()` child. Each tap node becomes a SPQR terminal.

```rust
let max_ratio = taps.iter().cloned().fold(1.0_f64, f64::max);
delay_line.ensure_tap_capacity(max_ratio);
```

The runtime body is `pedalkernel_rt::elements::DelayLine`. `bind_delay_runtime` upserts `ControlTarget::DelayTime` and `ControlTarget::DelayFeedback`, and adds an `LfoBinding` targeting `ModulationTarget::DelaySpeed` for wow/flutter when an LFO is wired to the delay's modulation port.

### BBD

`BbdBlock` handles `"BBD delay"`. The runtime struct (`BbdDelayLine` with model variants `mn3207`, `mn3007`, `mn3005`) lives in `pedalkernel-rt/src/elements/nonlinear/bbd.rs` and is unchanged from before the DspBlock refactor — what changed is the wiring. The hand-written BBD path that used to live inside `spqr_build` moved into `bbd_lowering`, and the union-find behavioural-gap splitter (now reused by other blocks) was extracted to `split_groups_at_behavioral_gaps`.

`bind_bbd_runtime` upserts `ControlTarget::BbdClockRate` and `BbdFeedback`, builds an `LfoBinding` targeting `ModulationTarget::BbdClock { bbd_idx }`, and walks the resistive divider that sets the BBD's clock voltage (`resolve_bbd_clock` recursively follows R/pot 2-terminals). It also BFS-detects wet/dry pots so they become bound controls.

### VCO

`VcoBlock` handles `"VCO"`. It's a generator — empty audio inputs, optional CV/Sync inputs, one or more audio outputs. The runtime body is `pedalkernel_rt::elements::Vco` from `oscillator.rs`: PolyBLEP saw / triangle / pulse with slow LP-filtered xorshift drift, thermal drift, a 12 kHz output low-pass, and a 303-flavoured saw curvature (0.92) and pulse width (0.48).

```rust
if let Some(p) = resolve_port(graph, &id, VCO_CV_PINS, PortRole::Cv) {
    inputs.push(p);
}
```

`split_groups_at_behavioral_gaps` skips VCO instances, and `all_generator_output_nodes` seeds downstream stage feedforward from the VCO's wave output. A `vco()` with no wired wave output is a compile error.

### VCA

`VcaBlock` handles `"VCA"`. Produces a `VcaBinding` with `input_node_id` and `output_node_id` set to `usize::MAX` — that's the sentinel meaning "this gain sits in the serial chain", applied after the WDF stage loop alongside the BBD wet path because gain commutes with the linear stages around it.

`bind_vca_runtime` walks every `EnvelopeFollower` component and binds `EF.out → VCA.cv` via `ModulationTarget::VcaCv`. The envelope tap that the VCA reads is resolved by `bind::resolve_envelope_tap` (the F4/F5 detector-tap logic that also drives the LA-2A side-chain).

### Spring reverb

`SpringBlock` handles `"spring reverb"` as a 1-in / 1-out insert. The runtime model is the Välimäki / Parker JAES 2010 spring tank — *not* a delay tap with diffusion. Each spring line is a cascade of M = 32 stretched first-order Schroeder allpasses inside a feedback loop with a fractional-interpolated transit delay, a one-pole damping low-pass, and a feedback gain `g = 10^(−3·Td/RT60)`:

```rust
let y = -a * x + x_km + a * y_km;   // stretched allpass
self.ap_x[slot] = x;  self.ap_y[slot] = y;  x = y;
```

`SpringModel::{type4, type8, type9, re201_tank}` carry up to 6 detuned springs in parallel, each with its own `transit_ms`, `fc_hz`, and `rt60_s`. The outputs are summed and passed through a 40 Hz DC blocker. Controls: `SpringDwell`, `SpringDecay`, `SpringDamping`, `SpringMix`. Each instance has its own `wet_mix` (unlike BBD, where wet/dry is shared — a deliberate fix during the spring reverb landing).

## Runtime primitives

The DSP blocks lean on a small set of zero-allocation runtime primitives that are also reused by the synth voice layer:

- **`oscillator::Vco`** — the PolyBLEP body the `VcoBlock` instantiates. `set_base_freq` + `set_cv_pitch` (1 V/oct) drive `phase_inc` at audio rate without recomputing the WDF scattering solve.
- **`envelope::DecayEnvelope`** — one-pole exponential decay with slide-aware `trigger_if` and an accent-override path through `set_decay_coeff`. The 303 MEG/VEG envelope distinct from the `AdsrEnvelope` that the VCA carries.
- **`glide::Glide`** — one-pole portamento (`current = coeff·current + (1−coeff)·target`) with gate-aware `activate` / `snap_to_target` / `trigger_if`. Not yet wired to a DspBlock — used directly by the synth voice path.

All three are `no_std`-friendly and guard against NaN / Inf inputs (the spring tank flushes its loop state, the VCO clamps, the envelope decays toward zero).

## Adding a new block

End-to-end:

1. **Implement the trait.** Write a unit struct in `compiler/<name>_lowering.rs` implementing `DspBlock`. Decide its port signature (`BlockIo`) and its runtime binding type.
2. **Register it.** Add it to the static registry in `compiler/dsp_block.rs::dsp_blocks()`. The registry is `&'static [&'static dyn DspBlock]`; one line.
3. **Add to the runtime-island gate.** If the block must have a binding for the circuit to be correct, list its `type_tag` in `RUNTIME_DSP_ISLAND_TAGS` so `reject_unlowered_behavioral` catches the silent-ship case.
4. **Build the runtime body.** Add the struct, controls, and modulation targets it needs in `pedalkernel-rt`. Keep `process()` allocation-free and bounded-time.

No change to `spqr_build`, no change to `compile_via_spqr_with_options`. The registry walks itself.

## See also

- [How it works](./how-it-works.md) — guide-level mention of where DSP blocks sit in the pipeline.
- [Compiler internals](./compiler-internals.md) — SPQR routing, terminal seeding, and the interplay between DSP blocks and the rest of the compile pass.
- [Nonlinear elements](./nonlinear-elements.md) — the catalogue of which DSP blocks carry state vs which are memoryless (none of them are — every DspBlock is stateful by nature).
