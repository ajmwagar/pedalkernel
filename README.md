<div align="center">
  <img src="logo/pedalkernel-logo.png" alt="PedalKernel Logo" width="200">
  <h1>PedalKernel</h1>
  <p><strong>Rust WDF kernel for FX pedals</strong></p>
  <p>Wave Digital Filter framework for modeling analog guitar effects pedals in real-time</p>
  
  [![Rust](https://img.shields.io/badge/rust-%23000000.svg?style=for-the-badge&logo=rust&logoColor=white)](https://www.rust-lang.org/)
  [![JACK](https://img.shields.io/badge/JACK-Audio-ff6b6b?style=for-the-badge&logo=audio-technica&logoColor=white)](https://jackaudio.org/)
  [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg?style=for-the-badge)](https://opensource.org/licenses/MIT)
</div>

---

## 🎛️ What is PedalKernel?

PedalKernel is a **Wave Digital Filter (WDF)** framework written in Rust for modeling analog guitar effects pedals in real-time. It provides a modular, type-safe foundation for building digital replicas of classic analog circuits.

### Why WDF?

Wave Digital Filters offer:
- **Physical accuracy** — Models actual circuit behavior, not approximations
- **Stability** — Guaranteed stable under all conditions
- **Modularity** — Connect circuit elements like building blocks
- **Real-time performance** — Suitable for live audio processing

---

## ✨ Features

### Core Framework
- 🔧 **WDF Elements** — Resistors, capacitors, inductors, voltage sources
- 🔌 **Adaptors** — Series and parallel connection topologies
- 🌳 **Tree Structure** — Composable circuit graphs
- ⚡ **Real-time Safe** — Zero-allocation audio paths

### Circuit Components
- 🎚️ **Resistors** — Ideal and non-linear (temperature-dependent)
- ⚡ **Capacitors** — With accurate charge/discharge models
- 🌀 **Inductors** — Including saturation effects
- 🔌 **Voltage Sources** — Ideal and impedance-matched

### Built-in Pedals
| Pedal | Description | Status |
|-------|-------------|--------|
| **Overdrive** | Tube Screamer-style soft clipping | ✅ Working |
| **FuzzFace** | Germanium fuzz with harsh distortion | ✅ Working |
| **Delay** | Analog-style bucket-brigade delay | ✅ Working |

---

## 🚀 Quick Start

### Prerequisites

- [Rust](https://rustup.rs/) 1.75+
- [JACK Audio Connection Kit](https://jackaudio.org/)

### Installation

```bash
# Clone the repo
git clone https://github.com/ajmwagar/pedalkernel
cd pedalkernel

# Build in release mode
cargo build --release

# Run tests
cargo test
```

### Running Examples

```bash
# Overdrive pedal
cargo run --example overdrive

# Fuzz pedal  
cargo run --example fuzz

# Delay pedal
cargo run --example delay
```

---

## 🏗️ Architecture

```
┌─────────────────────────────────────────┐
│           PedalKernel                   │
│  ┌─────────────────────────────────┐    │
│  │      WDF Framework              │    │
│  │  ┌─────┐ ┌─────┐ ┌─────────┐   │    │
│  │  │Root │ │Leaf │ │Adaptors │   │    │
│  │  └─────┘ └─────┘ └─────────┘   │    │
│  └─────────────────────────────────┘    │
│  ┌─────────────────────────────────┐    │
│  │      Circuit Elements           │    │
│  │  Resistor Capacitor Inductor    │    │
│  │  Diode    OpAmp      Transformer│    │
│  └─────────────────────────────────┘    │
│  ┌─────────────────────────────────┐    │
│  │      Pedal Examples             │    │
│  │  Overdrive  Fuzz  Delay         │    │
│  └─────────────────────────────────┘    │
└─────────────────────────────────────────┘
```

---

## 🎸 Usage Examples

### Creating an Overdrive Pedal

```rust
use pedalkernel::{WdfTree, Resistor, Capacitor, Diode};
use pedalkernel::pedals::Overdrive;

fn main() {
    // Create an overdrive pedal
    let mut pedal = Overdrive::new();
    
    // Set parameters
    pedal.set_gain(0.7);   // 0.0 - 1.0
    pedal.set_tone(0.5);   // 0.0 - 1.0
    pedal.set_level(0.8);  // 0.0 - 1.0
    
    // Process audio samples
    let input_sample = 0.5;
    let output = pedal.process(input_sample);
    println!("Output: {}", output);
}
```

### Creating a Fuzz Pedal

```rust
use pedalkernel::pedals::FuzzFace;

fn main() {
    let mut fuzz = FuzzFace::new();
    
    // Crank the fuzz
    fuzz.set_fuzz(0.9);
    fuzz.set_volume(0.8);
    
    // Process with harsh clipping
    let output = fuzz.process(input_sample);
}
```

### Building a Custom Circuit

```rust
use pedalkernel::elements::{Resistor, Capacitor, Inductor};
use pedalkernel::tree::{SeriesAdaptor, WdfTree};

// Build an RLC circuit
let resistor = Resistor::new(1000.0);  // 1kΩ
let capacitor = Capacitor::new(1e-6);  // 1µF
let inductor = Inductor::new(0.1);      // 100mH

// Connect in series
let rlc = SeriesAdaptor::new(
    SeriesAdaptor::new(resistor, capacitor),
    inductor
);

// Create the WDF tree
let mut tree = WdfTree::new(rlc, 48000.0);

// Process samples
let output = tree.process(input);
```

---

## 🔌 JACK Audio Integration

PedalKernel uses JACK for real-time audio processing:

```rust
use pedalkernel::AudioEngine;
use pedalkernel::pedals::Overdrive;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create your pedal
    let pedal = Overdrive::new();
    
    // Initialize JACK engine
    let engine = AudioEngine::new("OverdrivePedal", pedal)?;
    
    // Start processing (blocks until interrupted)
    engine.run()?;
    
    Ok(())
}
```

Connect to your DAW or audio interface using JACK's routing tools.

---

## 🗺️ Roadmap

- [ ] **More Pedals** — Phaser, Chorus, Reverb, Wah
- [ ] **VST3 Plugin** — DAW integration
- [ ] **LV2 Plugin** — Linux plugin format
- [ ] **Circuit Import** — SPICE netlist → WDF conversion
- [ ] **GUI** — Visual pedalboard designer
- [ ] **Profiling** — CPU usage optimization

---

## 📚 Documentation

- [API Documentation](https://docs.rs/pedalkernel) — Rust docs
- [WDF Theory](https://www.ece.rutgers.edu/~orfanidi/ece521/notes.pdf) — Background on Wave Digital Filters
- [Examples](examples/) — More usage examples

---

## 🤝 Contributing

Contributions welcome! Areas where help is needed:

- More circuit element models
- Additional pedal implementations
- Performance optimizations
- Documentation improvements
- Bug reports and testing

See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

---

## 📄 License

MIT © [Avery Wagar](https://github.com/ajmwagar)

---

<div align="center">
  <p><em>Built with 🦀 Rust for 🎸 guitarists</em></p>
</div>
