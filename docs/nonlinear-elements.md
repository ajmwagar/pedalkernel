---
title: "Nonlinear elements"
description: "Catalogue of nonlinear devices — which are memoryless, which carry state, and which are K-method candidates."
section: "Internals"
weight: 89
source_commit: "222212fe33f0aa223f7d545d890a16654c17cca8"
watches:
  - pedalkernel-rt/src/elements/nonlinear/
  - pedalkernel-rt/src/elements/modulation.rs
  - pedalkernel-rt/src/elements/controlled.rs
  - pedalkernel-rt/src/stage.rs
  - pedalkernel/src/compiler/k_method.rs
  - pedalkernel/src/compiler/component.rs
  - pedalkernel/src/compiler/dsp_block.rs
---

# Nonlinear Elements

The WDF engine solves a different equation per nonlinear device, and the *kind* of nonlinearity decides which solver runs and whether a lookup-table fast path is available. This page catalogues every nonlinear element the runtime knows about, classifies each as **memoryless** or **stateful**, and reports its K-method candidacy.

The distinction matters because the [K-method](./compiler-internals.md#k-method-tables) replaces per-sample Newton-Raphson with a precomputed table — but only for memoryless devices. A device whose output depends on its history (a BBD, an envelope follower, a photocoupler with thermal lag) cannot be tabulated as a pure function of its current input.

## What "memoryless" means here

A WDF nonlinear root is **memoryless** if its reflected wave `a_root` is a pure function of the incident wave `b_tree` and (optionally) a control voltage:

```
a_root = f(b_tree)               // 1D — diodes, zeners
a_root = f(b_tree, V_control)    // 2D — BJT (Vbe), triode (Vgk), JFET (Vgs)
```

No previous-sample state, no integrator, no charge buffer between calls. The root function can be sampled at compile time and replaced with a table lookup at runtime.

A device is **stateful** if its output depends on history independently of the WDF tree's incident wave — a BBD's bucket fill, an envelope follower's filtered RMS, a photocoupler LDR cell that cools off over tens of milliseconds. These devices cannot be tabulated; they have to run their internal update step per sample.

Note that being memoryless at the root does not mean the *circuit* is memoryless. Capacitors and inductors carry state inside the WDF tree as ordinary leaves; the diode at the root is still a memoryless function of the wave that arrives at it.

## Memoryless devices (K-method eligible)

These compile to a WDF root whose I-V curve is sampled into a [`KTable`](./compiler-internals.md#k-method-tables) at compile time. At runtime the root lookup is bilinear interpolation rather than Newton-Raphson iteration.

| Device | `RootKind` variant(s) | K-table dims | Solver | Notes |
|---|---|---|---|---|
| Single diode | `SingleDiode`, `ExplicitSingleDiode` | 1D | Wright Omega (explicit) | Per-device `Is` / `n` from datasheet |
| Anti-parallel diode pair | `DiodePair`, `ExplicitDiodePair` | 1D | Wright Omega (explicit) | Symmetric soft clipper (Tube Screamer, RAT) |
| Zener | `Zener` | 1D | Wright Omega | Breakdown clamp |
| NPN / PNP BJT | `Bjt(BjtRoot)` | 2D (`b_tree` × `Vbe`) | Newton-Raphson on Gummel-Poon | `is_pnp` flag toggles polarity |
| N- / P- JFET | `Jfet` | 2D (`b_tree` × `Vgs`) | Newton-Raphson, square-law | |
| N- / P- MOSFET | `Mosfet` | 2D (`b_tree` × `Vgs`) | Newton-Raphson, square-law | |
| Triode (12AX7, 12AU7, ...) | `Triode` | 2D (`b_tree` × `Vgk`) | Newton-Raphson on Koren | Per-instance `vgk_bias` field |
| Pentode (EL34, 6L6, ...) | `Pentode` | 3D candidate (`b_tree` × `Vg1k` × `Vg2k`) | Newton-Raphson on Koren | 3D candidacy reported, 3D tables not generated yet |

The K-method dispatch is single-line at the WDF stage level:

```rust
let a_root = if let Some(ref table) = k_table {
    if table.dims == 1 { table.lookup_1d(b_tree) }
    else { table.lookup_2d(b_tree, ctrl) }
} else {
    root.process(b_tree, rp)   // NR fallback
};
```

A stage either has a `KTable` (table lookup) or doesn't (NR fallback). The two paths are equivalent in the limit of infinite table resolution and converged NR; the table trades floating-point ops for L1 cache bandwidth.

### Eligibility gating

K-table generation is gated by `RootKind::k_method_candidacy()`, which mirrors the `Component::k_method_candidacy()` trait method:

```rust
match self {
    RootKind::DiodePair(_) | RootKind::SingleDiode(_)
    | RootKind::ExplicitDiodePair(_) | RootKind::ExplicitSingleDiode(_)
    | RootKind::Zener(_) => (true, 1),
    RootKind::Jfet(_) | RootKind::Triode(_)
    | RootKind::Mosfet(_) | RootKind::Bjt(_) => (true, 2),
    RootKind::Pentode(_) => (true, 3),
    _ => (false, 0),
}
```

A device-family override on the trait carries one source of truth from compile time (where the table generator decides whether to sample) to runtime (where the dispatch decides whether to look up). Devices missing from either match arm fall through to NR.

## Memoryless devices (not K-method candidates)

These devices are memoryless at the root but excluded from K-method tabulation for other reasons:

| Device | `RootKind` | Why not tabulated |
|---|---|---|
| Op-amp root | `OpAmp(OpAmpRoot)` | Closed-loop dynamics interact with the post-processor (`NonIdealFxState`); gain changes with each pot move |
| OTA (CA3080) | `Ota(OtaRoot)` | Transconductance `gm` is an envelope-modulated parameter — the curve shape changes at audio rate |
| Vari-mu triode | `VariMu(TriodeRoot)` | Bias point drifts with rectified signal envelope (compressor topology) — curve translates per sample |
| JFET as variable resistor | `JfetVr(JfetRoot)` | `Rds` is a continuously-varying parameter, not a fixed I-V curve |

All four would in principle be tabulatable if the modulated parameter were treated as a third dimension — but the table size grows multiplicatively, and these devices already amortise well in NR (the OTA gm is plain `tanh`, etc.). The benefit/cost balance landed against tabulation.

## Stateful devices

These devices carry internal state that updates per sample, independently of the WDF root function. They participate in the circuit as ordinary components but cannot be tabulated.

| Device | Where it lives | State carried |
|---|---|---|
| Bucket-brigade delay (BBD) | `elements/nonlinear/bbd.rs` | Per-bucket charge, NE571 companding state, clock feedthrough phase |
| Delay line / tap | `elements/nonlinear/delay.rs` | Ring buffer of past samples |
| Spring reverb tank | `elements/nonlinear/spring.rs` | Per-spring transit delay, M=32 allpass states, damping filter, DC blocker |
| Jiles-Atherton transformer core | `elements/nonlinear/jiles_atherton.rs` | `(H, Hdot, M, Mdot)` magnetisation state — full B-H hysteresis |
| LFO | `elements/modulation.rs` | Oscillator phase |
| Envelope follower | `elements/modulation.rs` | One-pole filtered RMS with attack/release |
| Photocoupler (Vactrol) | `elements/controlled.rs` | LED current, LDR resistance (thermal lag, ~10–100 ms) |
| VCO / VCF / VCA | `elements/synth.rs`, `oscillator.rs` | Oscillator phase, filter state, gain envelope |
| ADSR / DecayEnvelope | `elements/synth.rs`, `envelope.rs` | Stage + per-stage timer |
| Glide (portamento) | `glide.rs` | One-pole pitch tracking with gate state |
| Slew limiter | `elements/nonlinear/slew.rs` | Previous output sample |

The BBD and the photocoupler are the two cases where "stateful" matters the most: the BBD because its tone literally is its history (companding, leakage, clock feedthrough), and the photocoupler because its thermal response shapes everything an optical compressor does. Neither has a K-method shortcut available even in principle.

### How stateful devices are routed

Stateful devices do not become `RootKind` variants. They take one of two paths through the compiler:

- **WDF leaves with state** — `Photocoupler`, `JfetVariableResistor`, `LeakyCapacitor`, `SwitchedResistor`, `UnitDelay`, **`JaMagnetizing`** all land as variants of `LeafKind` in `pedalkernel-rt::wdf_leaf`. They sit inside a WDF tree as ordinary one-ports; their `process()` method advances internal state per sample. The Jiles-Atherton core, in particular, replaces the linear `DynNode::Inductor` magnetizing branch of a transformer T-equivalent when the DSL config provides JA parameters (`ja_ms`, `ja_a`, `ja_alpha`, `ja_k`, `ja_c`).
- **DSP blocks** — BBD, delay line / tap, VCO, VCA, spring reverb are lowered by the [DspBlock](./dsp-blocks.md) registry rather than reduced through SPQR. They get their own per-instance binding on the `CompiledPedal` and execute as a separate per-sample pass.

LFOs, envelope followers, glide, and ADSR are not standalone components — they're modulation sources or runtime primitives consumed by other elements (a DspBlock's `bind_runtime` step wires them in).

## Adding a new nonlinear device

Three places to touch when introducing a new memoryless nonlinearity:

1. **The root type.** Add a variant to `pedalkernel-rt::stage::RootKind` and a `*Root` struct implementing `process(b_tree, rp) -> f64`. Include a per-instance bias field (see [per-instance bias analysis](./compiler-internals.md#per-instance-bias-analysis)) if the device has a DC operating point.
2. **K-method candidacy.** Add the new variant to both `RootKind::k_method_candidacy()` and the `Component::k_method_candidacy()` override on the DSL component. Decide whether it's 1D (single-port) or 2D (control-modulated).
3. **The build factory.** `pedalkernel/src/compiler/build.rs` constructs the right `RootKind` from the parsed component; add the case there.

Adding a stateful device skips the K-method step — it's a component with its own per-sample update, exposed via the ordinary `Component` trait.

## See also

- [Compiler internals: K-method tables](./compiler-internals.md#k-method-tables) — generation algorithm, table sizing, runtime dispatch.
- [Compiler internals: per-instance bias analysis](./compiler-internals.md#per-instance-bias-analysis) — how DC operating points are derived for the 2D K-tables.
- [The Component trait](./component-trait.md) — the trait method (`k_method_candidacy()`) that compile-time gating consults.
- [Modeling limits](./modeling-limits.md) — what each device's underlying model captures and where it approximates.
