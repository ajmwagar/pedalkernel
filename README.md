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

PedalKernel is a circuit-to-audio compiler. You describe a pedal the way you'd draw it on a napkin — resistors, caps, diodes, tubes, pots, with real values and real wiring — and PedalKernel compiles it into a real-time audio engine using Wave Digital Filters.

Every component shapes the sound the way the physical part would. A 220nF cap in the feedback loop rolls off differently than a 100nF. Germanium diodes clip softer than silicon. There are no "models" or "algorithms" to choose from — just the circuit.

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
```

## Documentation

Full documentation — DSL reference, API docs, roadmap, architecture notes — lives at **[docs.pedalkernel.com](https://docs.pedalkernel.com)**.

## Contributing

Contributions welcome. See [CONTRIBUTING.md](CONTRIBUTING.md) and the [CLA](CLA.md).

## License

AGPLv3 © [Avery Wagar](https://github.com/ajmwagar)
