---
title: "Controls and pots"
description: "How runtime control updates flow through the engine — binding, dispatch, and what each pot movement actually costs."
section: "Internals"
weight: 87
source_commit: "2cafa26fe49ea6ad3d1ccf9f52401060c4ae1ea2"
watches:
  - pedalkernel/src/compiler/spqr_control.rs
  - pedalkernel/src/compiler/compiled.rs
  - pedalkernel/src/compiler/stage.rs
  - pedalkernel/src/compiler/bind.rs
  - pedalkernel/src/elements/nonlinear/opamp.rs
  - pedalkernel/src/compiler/rigid/opamp_root.rs
---

# Controls and Pots

> **Preview.** Several of the paths described here (`spqr_control::bind_controls`, `BlackFeedbackStage::set_pot`, the OpAmpRoot feedback-pot pathway) are on `feature/spqr-tree` and not yet on `main`. The user-visible API (`pedal.set_control("Drive", 0.7)`) is stable on both branches.

When a user moves a knob, a lot of math has to happen before the next sample comes out right. This page walks through exactly what — the compile-time binding, the runtime dispatch, the per-stage recompute paths, and the resulting heat map of "what is actually expensive."

The goal is specific enough that a contributor can decide, when adding a new feature, whether they're spending a cheap operation or a transcendental one.

## Compile-time binding

At compile time, every `controls { ... }` entry in the `.pedal` file is bound to the stage(s) that hold the underlying pot. The binding lives in `pedalkernel/src/compiler/spqr_control.rs`.

`bind_controls()` walks each control and calls `find_pot_binding()` to produce a `ControlBinding`:

```rust
pub(super) struct ControlBinding {
    pub label: String,               // user-visible name: "Drive", "Tone"
    pub target: ControlTarget,       // which stage, and how to reach the pot
    pub component_id: String,        // pot component ID in the circuit
    pub component_id_aw: String,     // precomputed "{id}__aw"  (Baxandall split)
    pub component_id_wb: String,     // precomputed "{id}__wb"
    pub max_resistance: f64,
    pub taper: PotTaper,
    pub range: (f64, f64),
}
```

`ControlTarget` encodes "which kind of stage, at what index":

```rust
pub(super) enum ControlTarget {
    PotInStage(usize),                  // WDF stage index
    PotInIirStage(usize),               // IIR biquad stage index
    PotInMultiNlStage(usize, usize),    // (multi-NL stage, passive child index)
    PotInBlackFeedbackStage(usize),     // BlackFeedback stage index
    // ... plus LFO, Delay, Switch, etc.
}
```

The search is exhaustive at compile time: for WDF stages the code checks the main tree, the impedance-dividing `zf_child`, the `zg_child`, and every opamp child. For multi-NL stages it scans `passive_children`. The resulting `Vec<ControlBinding>` is stored on `CompiledPedal` and never touched again at runtime.

Two key compile-time optimisations worth knowing about. First, the `__aw` / `__wb` component-id strings used for Baxandall-split pots are computed once during binding — the runtime path never formats strings. Second, the binding records include the taper (`PotTaper::BTaper` for audio potentiometers, for example), so taper mapping is applied on the RT thread without a lookup.

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

So: one or two adds, a multiply, one or two divisions per node. A typical WDF tree has 5–20 nodes. Total cost per pot change is on the order of 20–60 floating-point operations. **Zero allocation.** The full scattering-matrix recompute is more expensive (it's linear in the tree size but involves more per-node math) and is throttled — batched every 32 samples by `flush_recompute()` rather than run per sample.

### IIR stages

An IIR stage compiled from a tone stack (active EQ, passive tone control) keeps its biquad coefficients inline. When a pot moves, the coefficients are re-derived from scratch. This is the most expensive per-sample pot path:

```rust
let f0 = 1.0 / (2.0 * PI * (r_series_product * c_shunt_product).sqrt());
let q  = r_fb / (r_fb - r_crit);
let w0 = 2.0 * PI * f0 / sample_rate;
let sin_w0 = w0.sin();
let cos_w0 = w0.cos();
let alpha = sin_w0 / (2.0 * q);
// ... five biquad coefficient derivations ...
```

Three transcendentals (`sqrt`, `sin`, `cos`) plus around twenty multiplies/divides per pot movement. If the stage is a simple DC-gain-only IIR (no caps), the cost drops to three scalar operations. But the common case — a wah or tone control — is the full biquad path.

There is no fast path for tone-stack pots today. That's the single biggest opportunity in this module: batch the biquad recompute on a 32-sample schedule the way WDF scattering matrices are batched.

### Multi-NL stages

Multi-nonlinear stages (differential pairs, pentodes in feedback with other nonlinear devices) solve a small MNA system each sample. When a pot changes, the scattering matrix has to be re-inverted — `O(N³)` in the MNA node count.

This is where the compiler's throttling earns its keep. The matrix inversion runs every 32 samples, not every sample. At 48 kHz that's roughly 670 µs between inversions, and a small matrix inverts in a few hundred microseconds, so the amortised per-sample cost is negligible.

For single-pot multi-NL stages the compiler can pre-compute an interpolation table indexed by pot position. That's `O(1)` lookup per sample — no matrix work at all.

### BlackFeedback stages

The simplest path. A `BlackFeedbackStage` with a pot on `Rf` just updates the stored resistance and calls `update_gain()`:

```rust
pub(super) fn set_rf(&mut self, rf: f64) {
    self.rf = rf;
    if self.stored_gbw > 0.0 {
        self.fx_state.update_gain(self.stored_gbw, self.gain(), self.sample_rate);
    }
}
fn gain(&self) -> f64 {
    if self.inverting { self.rf / self.ri.max(1.0) }
    else { 1.0 + self.rf / self.ri.max(1.0) }
}
```

Two or three scalar operations for the gain, plus an `exp()` call inside `update_gain()` to rebuild the GBW low-pass coefficient. Under ten operations total.

### OpAmpRoot feedback pots

When a WDF stage contains an `OpAmpRoot` whose feedback network has a pot, the WDF path above fires *and* `notify_pot_changed()` propagates:

```rust
let pot_r = self.tree.get_pot_resistance(pot_id)?;
let rf = pot_r + self.feedback_series_r;
let gain = rf / self.feedback_ri;
oa.set_gain(gain);                                // includes GBW re-coeff
```

Same order-of-magnitude as `BlackFeedback` — a handful of ops plus one `exp()`.

## The heat map

Ranked by per-sample cost during an active sweep, heaviest first:

| Stage kind | Ops / sample | Notes |
|---|---|---|
| **IIR resonant** (tone stack) | ~25 | 3 transcendentals (`sqrt`, `sin`, `cos`). Hot spot. |
| **Multi-NL scattering** | O(N³) but **throttled** | Inversion every 32 samples. Amortised cost low. |
| **WDF tree** | ~20–60 | Traversal of 5–20 nodes; arithmetic only. |
| **OpAmpRoot gain** | ~8 | One `exp()` for GBW coefficient. |
| **BlackFeedback gain** | ~8 | One `exp()` for GBW coefficient. |
| **IIR DC-only** | ~3 | Pure scalar math. |
| **Set-control dispatch** | ~50 cycles | String scan. Constant in knob count. |
| **Smoother step** | ~5 | One IIR step per active smoother. |

So the bottleneck, when it shows up, is IIR tone stacks. A pedal with one tone knob over a biquad is doing about 1.2 million float ops per second *just for the pot recompute*, mostly transcendental. At 48 kHz this is still well under 1% of CPU, but at 192 kHz with weak hardware and several tone knobs it adds up.

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

**`StateSpaceStage` pots are not yet bound.** The binding code has an explicit TODO:

```rust
Stage::StateSpace(_) => {
    // TODO: StateSpaceStage G-matrix delta
}
```

If a pot ends up inside a state-space stage, it currently has no effect at runtime. Any new state-space stage that wants pots will need the G-matrix delta path implemented first.

**IIR tone-stack recompute is not throttled.** Transcendentals run every sample a tone pot is moving. Moving this to the same 32-sample schedule as the multi-NL scattering recompute would cut the observable cost by roughly 32×, at the expense of slightly coarser temporal resolution on the pot sweep — probably inaudible below audio rate.

**The `set_control` lookup is linear.** Fine at N = 10; annoying at N = 100. If we ever get to composite pedalboards with combined control surfaces, a hash table would help.

**OpAmpRoot's feedback-pot search** uses `get_pot_resistance(pot_id)` which is a linear tree walk. One-per-opamp-feedback-pot-per-recompute, so rare; but with many nested opamp feedbacks in one stage it could add up.

## Where to look

- `compiler::spqr_control` — compile-time binding; `bind_controls`, `find_pot_binding`, `ControlBinding`.
- `compiler::compiled` — `CompiledPedal::set_control`, `SmoothedParam`, the dispatch.
- `compiler::stage` — `WdfStage::set_pot`, `IirStage::set_pot`, `BlackFeedbackStage::set_pot`, `MultiNlStage::set_pot`.
- `compiler::dyn_node` — `DynNode::recompute` (the WDF tree traversal).
- `elements::nonlinear::opamp` — `OpAmpRoot::set_feedback_pot_r`, `set_gain`, `compute_gbw_coeff`.

For the structural counterpart — how components declare that they have pot controls in the first place — see the [Component trait](./component-trait.md) page.
