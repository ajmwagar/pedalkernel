---
title: "Introduction"
description: "What PedalKernel is, what it isn't, and how to read the rest of this book."
section: "Guide"
weight: 5
---

PedalKernel is a circuit-to-audio compiler written in Rust. You describe a pedal, amp, or synth module the way you'd draw it on a napkin — resistors, caps, diodes, tubes, pots, with real values and real wiring — and PedalKernel compiles it into a real-time audio engine using [Wave Digital Filters](https://en.wikipedia.org/wiki/Wave_digital_filter). The same description also drives KiCad netlist export and a Mouser-ready bill of materials, so the file you listen to is the file you build.

## The premise

Most amp and pedal simulators model a _behaviour_ — a soft-clipping curve, an impulse response, a tanh waveshaper tuned to sound "like" the thing. PedalKernel models the _circuit_. Each component participates as itself. A 220 nF cap in the feedback loop rolls off differently than a 100 nF. Germanium diodes soften the edges; silicon clips harder. A 12AX7 saturates like a 12AX7, not like a "generic triode" with a warmth knob.

The cost is arithmetic: Newton-Raphson per nonlinear root, scatter up, scatter down, state update — every sample, every circuit, at the host sample rate. The payoff is that the same numerical model that produces the audio also produces the PCB.

## What's in the box

PedalKernel is a Cargo workspace with three public crates:

- **[`pedalkernel`](/api/pedalkernel/)** — the kernel. DSL parser, WDF compiler, real-time audio runtime, KiCad and BOM exporters, optional JACK and TUI front ends.
- **[`pedalkernel-layout`](/api/pedalkernel_layout/)** — schematic layout. Turns a parsed pedal into a positioned schematic for KiCad or plugin UIs.
- **[`pedalkernel-validate`](/api/pedalkernel_validate/)** — SPICE validation harness. Signal generation, spectral analysis, regression reports against ngspice ground truth.

Under `pedalkernel/examples/` the source tree ships a library of working circuits — overdrives, fuzzes, distortions, modulation, delay, tube amps, and a small synth voice — all as `.pedal` files you can read, modify, and rebuild.

## A quick look at the DSL

```
pedal "Tube Screamer" {
  components {
    R1: resistor(4.7k)       # These values are the tone
    C1: cap(220n)             # Change them and the sound changes
    D1: diode_pair(silicon)   # Si clips harder than Ge
    Gain: pot(500k)
  }
  nets {
    in -> C1.a
    C1.b -> R1.a, D1.a
    D1.b -> gnd
    R1.b -> Gain.a
    Gain.b -> out
  }
  controls {
    Gain.position -> "Drive" [0.0, 1.0] = 0.5
  }
}
```

This is the complete source for a working pedal. The [DSL reference](/book/dsl/) has the full grammar; the [Components reference](/book/components/) has every part type and variant.

## From tone to PCB

```
                      +---> WDF audio engine   (WAV / JACK real-time)
                      |
  .pedal file  ------>+---> KiCad netlist       (PCB layout)
                      |
                      +---> Bill of Materials   (Mouser-ready)
```

## Running it

```bash
git clone https://github.com/ajmwagar/pedalkernel
cd pedalkernel
cargo build --release
```

From there, the [Rust API](/book/api/) shows how to go from a `.pedal` file to a live processor, and the [CLI & tools](/book/tools/) page covers the Python helpers for schematic rendering and Mouser CSV generation.

## How to read this book

The **Guide** pages are narrative — start there if you want to understand how the engine works and what physical realism features it offers.

The **Reference** pages are lookups — DSL grammar, component catalogue, Rust API primer, performance numbers.

The **Internals** pages are for contributors and the curious: where the model is circuit-exact, where it approximates, and how we validate it against ngspice.

The **Project** pages cover where we're going next, and how to join in.
