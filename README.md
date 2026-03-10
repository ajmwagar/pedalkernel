<div align="center">
  <img src="logo/pedalkernel-logo.png" alt="PedalKernel Logo" width="200">
  <h1>PedalKernel</h1>
  <p><strong>Real components. Real circuits. Real tone.</strong></p>
  <p>Write a schematic in <code>.pedal</code> files. Hear it. Build it.</p>

  [![Rust](https://img.shields.io/badge/rust-%23000000.svg?style=for-the-badge&logo=rust&logoColor=white)](https://www.rust-lang.org/)
  [![JACK](https://img.shields.io/badge/JACK-Audio-ff6b6b?style=for-the-badge&logo=audio-technica&logoColor=white)](https://jackaudio.org/)
  [![License: AGPL v3](https://img.shields.io/badge/License-AGPLv3-yellow.svg)](https://opensource.org/licenses/agpl-v3)
</div>

---

## What is PedalKernel?

PedalKernel is a circuit-to-audio compiler. You describe a pedal the way you'd draw it on a napkin -- resistors, caps, diodes, tubes, pots, with real values and real wiring -- and PedalKernel compiles it into a real-time audio engine using Wave Digital Filters.

The same `.pedal` file exports to KiCad for PCB layout and generates a Mouser bill of materials. Design the tone first, then solder it.

Every component shapes the sound the way the physical part would. A 220nF cap in the feedback loop rolls off differently than a 100nF. Germanium diodes clip softer than silicon. A 12AX7 saturates differently than a 12AU7. There are no "models" or "algorithms" to choose from -- just the circuit.

## Quick example

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

## Quick start

```bash
git clone https://github.com/ajmwagar/pedalkernel
cd pedalkernel
cargo build --release
cargo test

# Render a pedal file to WAV
cargo run --example process -- pedalkernel/examples/pedals/overdrive/tube_screamer.pedal

# Interactive TUI -- tweak knobs in real time
cargo run --example tui --features tui -- pedalkernel/examples/pedals/overdrive/tube_screamer.pedal
```

## From tone to PCB

The same `.pedal` file drives three outputs:

```
                          +---> WDF audio engine ---> WAV / JACK real-time
                          |
  .pedal file ---> parse -+---> KiCad netlist ---> PCB layout
                          |
                          +---> Bill of Materials ---> Mouser order
```

## What's included

**12 pedals** spanning overdrive, fuzz, distortion, modulation, compression, delay:
- Tube Screamer, Blues Driver, Klon Centaur, Fulltone OCD (overdrives)
- Fuzz Face, Big Muff (fuzz)
- ProCo RAT (distortion)
- MXR Phase 90 (phaser)
- Boss CE-2 (modulation)
- Dyna Comp (compressor)
- Boss DM-2, Memory Man (delays)

**3 tube amps**:
- Tweed Deluxe 5E3, Bassman 5F6-A, Marshall JTM45

**3 synth modules**:
- CEM3340 VCO, Moog Ladder VCF, Minisynth (complete mono voice)

Full component reference and circuit details in [`docs/components.md`](docs/components.md).

## Feature flags

| Feature | Default | What it adds |
|---------|---------|-------------|
| `jack-rt` | Yes | JACK real-time audio -- sub-5ms latency |
| `tui` | Yes | Interactive ASCII control surface |
| `cli` | Yes | Command-line interface |
| `hardware` | No | BOM generation, voltage checks, `.pedalhw` specs |
| `gummel-poon` | Yes | Advanced BJT modeling with base charge effects |
| `debug-trace` | No | Detailed signal-level tracing for debugging |
| `solver-stats` | No | WDF solver statistics |
| `fault-injection` | No | Injected faults for test validation |
| `runtime-warnings` | No | Warnings for out-of-range device operation |
| `pro-pedals` | No | Tests that reference private pedalkernel-pro circuits |

Minimal DSP-only build:

```bash
cargo build --no-default-features
```

## Documentation

- **[Components Reference](docs/components.md)** — All available component types, syntax, and variants
- **[How it works](docs/how-it-works.md)** — WDF compilation pipeline and circuit solving
- **[Performance](docs/performance.md)** — Benchmarks, CPU usage, scalability
- **[Hardware export](docs/hardware.md)** — KiCad export, BOM generation, voltage checks
- **[Python tools](docs/tools.md)** — Schematic rendering, BOM generation
- **[Rust API](docs/api.md)** — Parsing, compilation, and real-time processing
- **[Physical realism](docs/physical-realism.md)** — How circuits match real hardware behavior
- **[UI manifests](docs/UI_MANIFESTS.md)** — Control surface definitions (for VST UI generation)

## Performance

All pedals run comfortably in real-time. Tube Screamer: 1.2% CPU at 48 kHz. Big Muff: 1.7%. Even the most complex circuits stay under 6% with room to spare. Full benchmark details in [`docs/performance.md`](docs/performance.md).

## License

AGPLv3 (c) [Avery Wagar](https://github.com/ajmwagar)
