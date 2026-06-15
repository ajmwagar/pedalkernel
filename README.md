<div align="center">
  <img src="logo/pedalkernel-logo.png" alt="PedalKernel Logo" width="200">
  <h1>PedalKernel</h1>
  <p><strong>Real components. Real circuits. Real tone.</strong></p>
  <p>Write a schematic in <code>.pedal</code> files. Hear it. Build it. Verify it.</p>

  [![CI](https://github.com/ajmwagar/pedalkernel/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/ajmwagar/pedalkernel/actions/workflows/ci.yml)
  [![SPICE-verified](https://img.shields.io/badge/SPICE--verified-≥90%25-success)](https://docs.pedalkernel.com/book/validation/)
  [![Rust 1.80+](https://img.shields.io/badge/rust-1.80%2B-orange)](https://www.rust-lang.org/)
  [![Docs](https://img.shields.io/badge/docs-docs.pedalkernel.com-blue)](https://docs.pedalkernel.com)
  [![License: AGPL v3](https://img.shields.io/badge/License-AGPLv3-yellow.svg)](https://opensource.org/licenses/agpl-v3)
</div>

---

## What is PedalKernel?

PedalKernel is a **circuit-to-audio compiler**. You describe a guitar pedal, amp, or synth module the way you'd draw it on a napkin — resistors, caps, diodes, tubes, pots, with real values and real wiring — and PedalKernel compiles it into a real-time audio engine using [Wave Digital Filters](https://en.wikipedia.org/wiki/Wave_digital_filter).

Every component participates as itself. A 220 nF cap in the feedback loop rolls off differently than a 100 nF. Germanium diodes soften the edges; silicon clips harder. A 12AX7 saturates like a 12AX7, not like a "generic triode" with a warmth knob. There are no behaviour models, no tanh waveshapers — just the circuit, solved per sample.

The same `.pedal` file drives:

- A real-time **audio engine** (WAV, JACK, or embedded `no_std` target).
- A **KiCad netlist** and a **Mouser-ready bill of materials** — the file you listen to is the file you build.
- A **SPICE validation harness** that compares engine output to ngspice ground truth on every PR.

## Quick example

```
pedal "Tube Screamer" {
  components {
    R1: resistor(4.7k)        # These values are the tone
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

That's the complete source for a working pedal. The DSL grammar is documented at [docs.pedalkernel.com/book/dsl](https://docs.pedalkernel.com/book/dsl/).

## Quick start

```bash
git clone https://github.com/ajmwagar/pedalkernel
cd pedalkernel
cargo build --release

# Process a WAV through a stock circuit:
cargo run --release --example process -- \
  pedalkernel/examples/pedals/overdrive/tube_screamer.pedal \
  in.wav out.wav Drive=0.7

# Or fire up the live JACK + TUI front end:
cargo run --release --bin tui -- \
  pedalkernel/examples/pedals/distortion/proco_rat.pedal
```

The `examples/pedals/` library ships circuit-accurate ports of common pedals — Tube Screamer, ProCo RAT, Big Muff, Klon Centaur, Fuzz Face, Boss DM-2, Boss CE-2, Memory Man, Phase 90, MXR Dyna Comp, Fulltone OCD, SD-1, Blues Driver — plus tube amps (Bassman 5F6-A, Marshall JTM45, Tweed Deluxe 5E3) and a small synth set (MiniMoog-style minisynth, CEM3340 VCO, Moog ladder VCF).

## What's in the workspace

PedalKernel is a Cargo workspace of four crates:

| Crate | Role |
|---|---|
| **`pedalkernel`** | DSL parser, WDF compiler, runtime glue, KiCad / BOM exporters, optional JACK + TUI front ends. |
| **`pedalkernel-rt`** | The `no_std` audio runtime — WDF stages, nonlinear roots, DSP blocks, metering. Embeddable. |
| **`pedalkernel-layout`** | Schematic layout. Turns a parsed pedal into a positioned schematic for KiCad or plugin UIs. |
| **`pedalkernel-validate`** | SPICE validation harness. Signal generation, spectral analysis, regression reports against ngspice ground truth. |

Real-time-critical code lives in `pedalkernel-rt`, which compiles `no_std + alloc` so the engine is ready for embedded targets. Production-quality bindings for VST3, VCV Rack, and Daisy Seed are available [commercially](#commercial-licenses-and-bindings).

## Documentation

Full documentation — DSL reference, Rust API, compiler internals, modeling limits, roadmap, validation methodology — lives at **[docs.pedalkernel.com](https://docs.pedalkernel.com)**.

Suggested starting points:

- [Introduction](https://docs.pedalkernel.com/book/introduction/) — what PedalKernel is and isn't.
- [How it works](https://docs.pedalkernel.com/book/how-it-works/) — the WDF compile pipeline at the guide level.
- [Compiler internals](https://docs.pedalkernel.com/book/compiler-internals/) — SPQR decomposition, blockwise multi-NL, K-method tables, cross-network coupling, per-instance bias analysis.
- [Modeling limits](https://docs.pedalkernel.com/book/modeling-limits/) — where it's circuit-exact, where it approximates, and why.
- [Validation](https://docs.pedalkernel.com/book/validation/) — how ngspice is used as ground truth.

## Continuous integration

Every push and PR runs the full matrix on GitHub Actions:

- **`spice-validation`** — the firm gate. Runs the engine against committed ngspice reference `.npy` files via `pedalkernel-validate`, and fails if the pass rate drops below `SPICE_MIN_PASS_PCT` (currently 90%). This is the physical-fidelity check that catches "the math drifted."
- **`test`** — the feature-matrix `cargo test` (`no features`, `tui only`, `jack-rt only`, `ebers-moll`, `all features`), gated at `TEST_MIN_PASS_PCT` (70%).
- **`clippy`** + **`fmt`** — lint and style, informational.
- **`examples`** — every example crate compiles.
- **`bench`** — Criterion benchmarks (`wdf_bench`), informational; flags > 20% regressions in PR comments and uploads results as artifacts.
- **`deny`** — `cargo-deny` for advisories, license policy, supply-chain bans, and source whitelisting.
- **`attribution`** — `cargo-about` renders the third-party-license bundle every commit so release builds always have a current attribution sheet.

The CI is structured so that the **firm gate is physical correctness** (SPICE) rather than per-test green. Style and flaky test rows stay informational so noise can't block a fidelity-correct PR.

## Contributing

Contributions welcome. See the [Contributing guide](https://docs.pedalkernel.com/book/contributing/). First-time PRs get a one-click CLA signing prompt from [CLA Assistant](https://cla-assistant.io); full text at [CLA.md](CLA.md).

Especially valued: new vintage circuits with public schematics, SPICE reference circuits, bug reports with `.pedal` repros, and contributions to the [roadmap items](https://docs.pedalkernel.com/book/roadmap/).

## Commercial licenses and bindings

PedalKernel is AGPLv3 — free to use, modify, and redistribute under the AGPL. The same source tree powers a commercial product line maintained by **[Puget Audio](mailto:info@puget.audio)** for teams that need closed-source distribution, prebuilt host bindings, or vendor support:

- **VST3 bindings** — plug PedalKernel circuits into any DAW (Ableton, Logic, Reaper, Bitwig, …) as a VST3 plug-in with multi-instance state, parameter automation, and DAW preset I/O.
- **VCV Rack bindings** — exposes any `.pedal` as a Rack module with full CV/audio I/O, polyphony, and Rack's preset/automation surface.
- **Daisy Seed runtime** — a tuned `no_std` build of `pedalkernel-rt` for the Electrosmith Daisy Seed family (Patch, Pod, Field), with hardware abstraction for the audio codec, knobs, and switches.
- **Commercial license + support** — a closed-source license for the kernel and runtime crates, plus priority bug fixes, feature work, and direct engineering support.

Contact **info@puget.audio** for licensing, pricing, and integration questions.

## License

AGPLv3 © [Avery Wagar](https://github.com/ajmwagar). Contributions are licensed to the project under the [CLA](CLA.md). Embedding the kernel or runtime in a closed-source product, or shipping the commercial bindings listed above, requires a commercial license from [Puget Audio](mailto:info@puget.audio).
