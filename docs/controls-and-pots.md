---
title: "Controls and pots"
description: "How runtime control updates flow through the engine — binding, dispatch, and what each pot movement actually costs."
section: "Internals"
weight: 87
source_commit: "222212fe33f0aa223f7d545d890a16654c17cca8"
watches:
  - pedalkernel/src/compiler/spqr_control.rs
  - pedalkernel/src/compiler/compiled.rs
  - pedalkernel/src/compiler/stage.rs
  - pedalkernel/src/compiler/bind.rs
  - pedalkernel-rt/src/elements/nonlinear/opamp.rs
  - pedalkernel/src/compiler/rigid/opamp_root.rs
  - pedalkernel/src/compiler/rigid/state_space.rs
  - pedalkernel/src/compiler/dsp_block.rs
---

# Controls and Pots

When a user moves a knob, a lot of math has to happen before the next sample comes out right. This page walks through exactly what — the compile-time binding, the runtime dispatch, the per-stage recompute paths, and the resulting heat map of "what is actually expensive."

The goal is specific enough that a contributor can decide, when adding a new feature, whether they're spending a cheap operation or a transcendental one.

## Compile-time binding

At compile time, every `controls { ... }` entry in the `.pedal` file is bound to the stage(s) that hold the underlying pot. The binding lives in `pedalkernel/src/compiler/spqr_control.rs`.

`bind_controls()` walks each control and calls `find_pot_binding()` to produce a `ControlBinding`:

```rust
pub(super) struct ControlBinding {
    pub label: String,               // user-visible name: "Drive", "Tone"
    pub target: ControlTarget,       // which stage / element, and how to reach it
    pub component_id: String,        // pot component ID in the circuit
    pub component_id_aw: String,     // precomputed "{id}__aw" — RT-side leaf id
    pub component_id_wb: String,     // precomputed "{id}__wb"
    pub max_resistance: f64,
    pub taper: PotTaper,
    pub range: (f64, f64),
}
```

`ControlTarget` is a wider enum than just "pot in a stage" — controls also reach LFOs, delays, BBDs, switches, sub-circuits, and triggers:

```rust
pub(super) enum ControlTarget {
    PotInStage(usize),                       // WDF stage index
    PotInMultiNlStage(usize, usize),         // (multi-NL stage, passive child)
    SidechainControl(usize),                 // forward to sidechain sub-circuit
    SubcircuitControl { subcircuit_idx: usize },
    LfoRate(usize),
    LfoDepth(usize),
    DelayTime(usize),
    DelayFeedback(usize),
    BbdClockRate(usize),
    BbdFeedback(usize),
    SwitchPosition { switch_id: String, num_positions: usize },
    Trigger(usize),
}
```

Pot-in-IIR and pot-in-BlackFeedback variants used to live here and are gone — IIR is no longer a top-level stage (it's a fast-path field inside `MultiNlStage`), and `BlackFeedbackStage` was retired in favour of plain `OpAmpRoot`-in-WDF.

The search is exhaustive at compile time: for WDF stages the code checks the main tree, the impedance-dividing `zf_child`, the `zg_child`, and every opamp child. For multi-NL stages it scans `passive_children`. The resulting `Vec<ControlBinding>` is stored on `CompiledPedal` and never touched again at runtime.

Two compile-time optimisations worth knowing about. First, three-terminal pots are split into two leaves at SPQR-build time so each half can sit independently in the WDF tree. The `__aw` and `__wb` component IDs name those halves and are precomputed into the binding so the RT path never formats strings. The split halves use linear taper internally (the user-facing taper is applied once in the binding), and a `complement: bool` flag on the GND-touching leaf inverts the position so the two halves always sum to `max_R`. Second, the binding records include the user-facing taper (`PotTaper::BTaper` for audio potentiometers, for example) so taper mapping is applied on the RT thread without a lookup.

## Runtime dispatch

`CompiledPedal::set_control(label, value)` in `pedalkernel/src/compiler/compiled.rs` is the entry point. The dispatch is deliberately simple:

```rust
pub fn set_control(&mut self, label: &str, value: f64) {
    let value = value.clamp(0.0, 1.0);
    for i in 0..self.controls.len() {
        if !self.controls[i].label.eq_ignore_ascii_case(label) {
            continue;  // linear scan, case-insensitive
        }
        let tapered = self.controls[i].taper.apply(value);
        let mapped = r0 + tapered * (r1 - r0);
        match &self.controls[i].target {
            ControlTarget::PotInStage(idx) => { /* ... */ }
            // ...
        }
    }
}
```

The scan is linear over the control vector. Typical pedals have three to ten controls, so this is five-ish string comparisons in the common case. No allocation. No hashing. Hashing would be a fine optimisation if control count ever grew large; it isn't today.

## The smoother

Raw knob positions are not applied directly. Each control runs through a one-pole exponential smoother with `tau = 10 ms`, so a step change spreads over roughly 480 samples at 48 kHz:

```rust
let tau = 0.010;
let coef = (-1.0 / (tau * sample_rate)).exp();   // ~0.9979 at 48 kHz
```

Every audio sample, each active smoother advances one step and calls `set_pot` on its bound stage. This is what makes audio-rate sweeps click-free: the recompute cost per sample is constant, but the recompute itself is gated to a slow glide.

## What each stage does on `set_pot`

The same abstract call — "this pot moved, update yourself" — resolves to very different work depending on stage kind. Here's what each one does.

### WDF stages

For a WDF stage whose tree contains a pot:

```rust
pub(super) fn set_pot(&mut self, comp_id: &str, value: f64) -> bool {
    let mut found = self.tree.set_pot(comp_id, value);
    // also check zf_child, zg_child, opamp_children
    if found {
        self.recompute_all();
        self.notify_pot_changed();  // opamp gain update
    }
    found
}
```

`recompute_all()` walks the WDF tree bottom-up and updates each adaptor's port resistance `rp` and balance coefficient `gamma`. The math per binary node is tiny:

```rust
// Series adaptor
*rp = r1 + r2;
*gamma = r1 / *rp;

// Parallel adaptor
let sum = r1 + r2;
*rp = r1 * r2 / sum;
*gamma = r2 / sum;
```

So: one or two adds, a multiply, one or two divisions per node. **Zero allocation.**

The cost story improved recently. `DynNode::Binary` now carries two new fields — `dirty: bool` and `has_dynamic: bool` — and pot updates only touch the path from the changed leaf up to its root, skipping any subtree that has no variable leaves at all. For a typical tone stack with one tone pot, that's `O(log n)` work per pot change instead of walking all 5–20 nodes. The full-walk path is still available as a fallback (compile-time setup, stages that haven't migrated). The full scattering-matrix recompute remains throttled — batched every 32 samples by `flush_recompute()` rather than run per sample.

`FeedbackConfig`, the older type that held precomputed op-amp gain coefficients centrally, is deprecated. The op-amp's dynamic feedback divider is now read off the live WDF tree via `notify_pot_changed()` + `recompute_incremental()`, so a feedback-path pot change goes straight from `set_pot_dirty()` on the leaf to gain re-derivation without crossing a stale cache.

### Multi-NL stages

Multi-NL stages — differential pairs, pentodes in feedback with other nonlinear devices, BJTs promoted to a 2-port group, vari-mu triodes promoted to a 3-port group — solve a small MNA system each sample. When a pot changes, the scattering matrix has to be re-inverted — `O(N³)` in the MNA node count.

This is where throttling earns its keep. The matrix inversion runs every 32 samples, not every sample. At 48 kHz that's roughly 670 µs between inversions, and a small matrix inverts in a few hundred microseconds, so the amortised per-sample cost is negligible.

If the multi-NL stage's passive children are linear-only and series-parallel reducible, the compiler precomputes an `iir: Option<IirData>` field with biquad coefficients; if they're rigid, it precomputes `state_space: Option<StateSpaceData>` with `(A, B, C, D)` matrices. When a pot inside that linear subset moves, the runtime updates those coefficients on the fast path instead of re-inverting the full scattering matrix. The biquad re-derivation is the heaviest *per-sample* path on this page — three transcendentals (`sqrt`, `sin`, `cos`) and roughly twenty multiplies/divides — and there's no further fast path for it today. Batching the recompute on a 32-sample schedule the way the scattering matrix is batched would cut the cost roughly 32× and is the obvious next optimisation.

For single-pot multi-NL stages with no other nonlinearity, the compiler can also precompute an interpolation table indexed by pot position. That's `O(1)` lookup per sample — no matrix work at all.

### OpAmp feedback pots

When a WDF stage contains an `OpAmpRoot` whose feedback network has a pot, the WDF path above fires *and* `notify_pot_changed()` propagates:

```rust
let pot_r = self.tree.get_pot_resistance(pot_id)?;
let rf = pot_r + self.feedback_series_r;
let gain = rf / self.feedback_ri;
oa.set_gain(gain);                                // includes GBW re-coeff
```

A handful of float ops plus one `exp()` inside `set_gain()` to rebuild the GBW low-pass coefficient.

## The heat map

Ranked by per-sample cost during an active sweep, heaviest first:

| Path | Ops / sample | Notes |
|---|---|---|
| **Biquad re-derivation** (multi-NL `iir` field, tone stack) | ~25 | 3 transcendentals (`sqrt`, `sin`, `cos`). The hot spot. |
| **Multi-NL scattering re-inversion** | O(N³) but **throttled** | Inversion every 32 samples. Amortised cost low. |
| **WDF tree recompute** (incremental) | O(log n) of dirty path | ~3 ops per dirty node. Static subtrees skipped via `has_dynamic`. |
| **OpAmpRoot gain** | ~8 | One `exp()` for GBW coefficient. |
| **DC-gain IIR field** (no caps) | ~3 | Pure scalar math. |
| **Set-control dispatch** | ~50 cycles | String scan. Constant in knob count. |
| **Smoother step** | ~5 | One IIR step per active smoother. |

So the bottleneck, when it shows up, is biquad re-derivation inside multi-NL stages compiled from tone stacks. A pedal with one tone knob over a biquad is doing about 1.2 million float ops per second *just for the pot recompute*, mostly transcendental. At 48 kHz this is still well under 1% of CPU, but at 192 kHz with weak hardware and several tone knobs it adds up.

The multi-NL scattering recompute looks bad on paper (`O(N³)`) but in practice it's the cheapest-per-sample of the lot because of throttling.

## Is audio-rate sweeping safe?

Yes, with caveats. The things that would otherwise break real-time safety are all handled:

- **No heap allocations on the RT path.** Component IDs are pre-formatted at compile time. Smoother state is pre-allocated. Coefficient updates are in-place.
- **No locks or atomic waits.** The smoother is a lock-free one-pole filter.
- **Bounded per-sample cost.** The worst case (everything moving at once, IIR tone stack, WDF tree, feedback opamp) is a few hundred floating-point operations.

At 48 kHz and 96 kHz there is plenty of headroom. At 192 kHz with multiple tone-stack pots active, the IIR recompute starts to matter — individual pedals still fit comfortably, but a pedalboard of half a dozen EQ-heavy pedals could bite.

The existing claim on `compiler-internals.md` that "pots can be swept at audio rate without glitching" is accurate. The reason it is accurate is because of the 10 ms smoother — without it, you would hear zipper noise on discrete knob positions even with zero CPU contention.

## Known tech debt

A few items worth flagging:

**IIR tone-stack recompute is not throttled.** Transcendentals run every sample a tone pot is moving. Moving this to the same 32-sample schedule as the multi-NL scattering recompute would cut the observable cost by roughly 32×, at the expense of slightly coarser temporal resolution on the pot sweep — probably inaudible below audio rate.

**The `set_control` lookup is linear.** Fine at N = 10; annoying at N = 100. If we ever get to composite pedalboards with combined control surfaces, a hash table would help.

**OpAmpRoot's feedback-pot search** uses `get_pot_resistance(pot_id)` which is a linear tree walk. One-per-opamp-feedback-pot-per-recompute, so rare; but with many nested opamp feedbacks in one stage it could add up.

## Fidelity fixes (2026-06)

Two control-fidelity bugs surfaced during the LA-2A rebuild and were resolved at the *binding* level rather than per-stage — worth flagging because the symptom looked like a per-stage problem but the fix landed in `spqr_control`.

**Stage-owned pot discovery.** Each runtime `Stage` variant now declares its own pot-target type — `Wdf → PotInStage`, `MultiNl → PotInMultiNlStage`, `Iir → PotInIirStage`, `BlackFeedback → PotInBlackFeedbackStage`, `Blockwise / SerialDelayedFeedback / StateSpace → PotInStage`. The compiler's `find_pot_bindings` asks each stage which kind of binding to use and threads the right target through. Pre-fix, IIR and StateSpace stages returned `None` and the binding fell through to a generic `PotInStage(0)` placeholder, so moving the pot at runtime did nothing.

**Defaults are applied immediately, not through the smoother.** The pre-fix loop called the smoothed `set_control` to apply each `ctrl.default`. The smoother is initialised to `ctrl.default`, so `is_settled()` was already true and the actual `set_control_pot` never fired — leaving WDF / IIR / StateSpace leaves at their compile-time stamp (a `PassiveRType` pot at 0.5, say, regardless of what the `.pedal` file declared). The fix is one line in `spqr_control.rs`:

```rust
for ctrl in &pedal.controls {
    compiled.set_control_immediate(&ctrl.label, ctrl.default);
}
```

`set_control_immediate` writes straight into the leaf, re-derives the scattering on a WDF leaf, and re-runs the IIR coefficient sweep for an `IirPotBinding { comp_id, max_r, fixed_series_r, ri, position, role, feedback_r, non_inverting }`. The StateSpace path stashes the rebuilt `MnaSystem` in `stage.recompute_mna` so a pot wiggle re-derives the SS matrices without touching the audio sample loop's dispatch.

## Controls inside DSP blocks

Some controls live outside the SPQR pot system entirely — they bind through the [DSP block](./dsp-blocks.md) registry. A `delay_line` carries `DelayTime` and `DelayFeedback`; a `BBD delay` carries `BbdClockRate` and `BbdFeedback`; a `VCA` reads its CV from a bound `EnvelopeFollower` via `ModulationTarget::VcaCv`; a `spring reverb` carries `SpringDwell`, `SpringDecay`, `SpringDamping`, `SpringMix`. Each block's `bind_runtime` step upserts the relevant `ControlTarget` variant into the same `controls` vector that pots populate, so `pedal.set_control("Delay Time", 0.7)` works regardless of whether the underlying machinery is a WDF pot, an IIR coefficient sweep, or a DSP-block parameter.

## Where to look

- `compiler::spqr_control` — compile-time binding; `bind_controls`, `find_pot_binding`, `ControlBinding`.
- `compiler::compiled` — `CompiledPedal::set_control`, `SmoothedParam`, the dispatch.
- `compiler::stage` — `WdfStage::set_pot`, `MultiNlStage::set_pot` (which delegates into the `iir` / `state_space` fast paths when populated).
- `compiler::dyn_node` — `DynNode::recompute` (the WDF tree traversal).
- `elements::nonlinear::opamp` — `OpAmpRoot::set_feedback_pot_r`, `set_gain`, `compute_gbw_coeff`.

For the structural counterpart — how components declare that they have pot controls in the first place — see the [Component trait](./component-trait.md) page.
