---
title: "Metering and the metrics ring buffer"
description: "Real-time-safe signal and state metrics — what's recorded, how the ring buffer works, and how a UI consumes it."
section: "Internals"
weight: 88
source_commit: "ba0372ed07318273d8d1a016ca9a572acc0a27df"
watches:
  - pedalkernel/src/metering.rs
  - pedalkernel/src/compiler/compiled.rs
---

# Metering and the metrics ring buffer

PedalKernel has an optional real-time metering layer that lets a UI, TUI, or external tool observe what's going on inside a running `CompiledPedal` without touching the audio thread. It's the hook you'd use to build a VU meter, a tube-glow shader, a compressor GR display, a stage-level scope, or a power-supply-sag indicator.

The code lives in `pedalkernel/src/metering.rs` (~500 lines). It's always compiled in, but zero-cost when disabled: the per-sample hot path skips a single `Option::is_some()` check when metering isn't enabled, and when it is enabled it never allocates.

## What's recorded

One snapshot — a `UiMetrics` struct — is published every block (every ~128 samples, so ~375 snapshots/sec at 48 kHz). That's comfortably above UI refresh rates (60 fps = ~16.7 ms) so the reader always has fresh data.

`UiMetrics` is `#[repr(C)] Copy`, ~200 bytes, and fits a fixed shape regardless of circuit complexity. It has four logical sections:

**Signal levels.** RMS and peak in dB for the stage input and output, plus `signal_min` / `signal_max` (per-block envelope) and `lfo_phase` (for modulation-source displays).

**Compression.** `gain_reduction_db` — sampled from the active compressor control voltage or, for tube-based compressors, the bias-shift deflection.

**Per-stage levels.** `stage_levels[16]` — normalized RMS at the output of each stage in the pipeline. Every stage kind participates (`WdfStage`, `MultiNlStage`, `OpAmpStage`, `PushPullStage`). `stage_count` reports how many are active.

**Device state.** `tube_plate_current[12]` (mA) and `tube_dissipation[12]` (W) for up to 12 tubes, ready to drive a glow shader. `supply_voltage` and `supply_sag` for a "B+ droops under load" meter. `transformer_flux[4]` — currently a stub (zeroed; transformer saturation isn't computed yet).

## The ring buffer

`MetricsRingBuffer` is a single-producer, single-consumer lock-free ring:

- Capacity: 16 frames. Power-of-2 so wrap is a bitmask (`idx & mask`), not a modulo.
- Time coverage: ~43 ms at the 375 Hz write rate. More than enough head-room for a 60 fps reader to always land on a fresh frame even under jitter.
- Allocation: `Box`'d once at construction. Never grows, never reallocates.
- Synchronisation: a single `AtomicUsize` write index.

## Write path

The audio thread does two things per sample: accumulate, and occasionally reduce.

Accumulation is cheap — cumulative sum-of-squares for RMS, max for peak, per-stage samples indexed by position in the stage vector. All branch-free after the initial `if let Some(ref mut acc) = self.metrics_accumulator` gate.

```rust
if let Some(ref mut acc) = self.metrics_accumulator {
    acc.accumulate_levels(input, output);
    for (i, &lvl) in stage_levels.iter().enumerate() {
        acc.accumulate_stage(i, lvl);
    }
    // tube plate current, LFO phase, supply voltage...
}
```

Every ~128 samples `acc.is_block_complete()` returns true, and the accumulator reduces itself into a `UiMetrics` (RMS via `sqrt(sum_sq / n)`, peak hold with slow decay, per-stage normalisation) and publishes into the ring buffer:

```rust
let metrics = acc.reduce();  // drains + resets accumulators
if let Some(ref buffer) = self.metrics_buffer {
    buffer.write(metrics);
}
```

The write itself is wait-free — one volatile store to the slot, one atomic increment on the write index with `Release` ordering:

```rust
let idx = self.write_idx.load(Ordering::Relaxed);
let slot = idx & self.mask;
unsafe { std::ptr::write_volatile(ptr.add(slot), metrics); }
self.write_idx.store(idx.wrapping_add(1), Ordering::Release);
```

No locks, no allocations, no system calls. `write_volatile` keeps the compiler from reordering the store past the index bump.

## Read path

Consumers (a UI, a TUI panel, an external tool over IPC) call `read_latest()` and get the most recently published `UiMetrics`:

```rust
pub fn read_latest(&self) -> UiMetrics {
    let idx = self.write_idx.load(Ordering::Acquire);
    if idx == 0 { return UiMetrics::default(); }
    let slot = (idx.wrapping_sub(1)) & self.mask;
    unsafe { std::ptr::read_volatile(ptr.add(slot)) }
}
```

`Acquire` on the read pairs with `Release` on the write so any data stores before the index bump are visible to the reader. The `read_volatile` matches the `write_volatile` on the other side.

The consumer API on `CompiledPedal`:

- `pedal.metrics_buffer() -> Option<Arc<MetricsRingBuffer>>` — shares the buffer handle, intended for UI threads to hold across samples.
- `pedal.read_metrics() -> UiMetrics` — convenience that reads from the latest slot or returns a default if the buffer is empty.

There is no blocking-wait or callback API today. Polling at display rate is the intended pattern.

## Stage integration

All four stage kinds feed `stage_levels[]`. The process loop samples the post-stage output into the accumulator regardless of stage kind, so the array covers WDF trees (with all `RootKind` variants — passive or nonlinear), multi-NL R-type stages (whether they take the IIR or state-space fast path or solve the full coupled NR), op-amp stages, and push-pull triode pairs uniformly. For a wah-and-delay pedal you'd see one level per stage in the order the compiler placed them.

Tube-specific fields are tapped from `RootKind::Triode` and `RootKind::Pentode` variants inside `Stage::Wdf`. For push-pull topologies both tubes are sampled independently so a glow shader can show them breathing out of phase.

## Enabling at runtime

Metering is opt-in. Creating a pedal leaves it off; you turn it on when you want data:

```rust
let mut pedal = compile_pedal(&def, 48_000.0)?;
pedal.enable_metering();  // allocates the accumulator and ring buffer

// ... later, on the UI thread:
let metrics = pedal.read_metrics();
draw_vu(metrics.output_rms_db);
```

When metering is disabled, `self.metrics_accumulator` is `None` and the per-sample path skips all accumulation.

## Known limitations

Four things worth flagging for contributors:

- **Transformer flux is a stub.** `transformer_flux[4]` is zeroed every block. Computing real core saturation requires exposing the transformer's internal flux state to the accumulator, which isn't wired yet. `// TODO: implement` sits at the responsible line.
- **Peak decay happens on the audio thread.** `peak *= 0.9995` per block means peak holds decay at ~73% per 10 ms. A UI layer should not apply its own decay on top — you'd end up double-smoothing and the meters would look sluggish.
- **Polling only.** No `wait_next_frame()` or callback API. A UI thread pinned to `vsync` is fine; a latency-sensitive consumer that needs "new data" signalling would have to build that on top.
- **Single consumer.** The SPSC design means only one reader. Multiple panels sharing a pedal need to share an `Arc<MetricsRingBuffer>` and pull from the same slot, or the architecture needs a broadcast layer.

No UI consumes this yet — the infrastructure landed ahead of the display work. If you're building a VST UI panel or a TUI meter, start with `pedal.metrics_buffer()` and a 60 Hz poll.

## Where to look

- `metering.rs` — types (`UiMetrics`, `MetricsAccumulator`, `MetricsRingBuffer`), constants (`MAX_TUBES`, `MAX_TRANSFORMERS`, `MAX_STAGES`), block-reduction math.
- `compiler/compiled.rs` — where the process loop accumulates per sample and triggers the block reduction. Look for `metrics_accumulator` and `metrics_buffer`.
