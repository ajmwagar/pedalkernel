# PedalKernel

> Rust WDF kernel for FX pedals

Wave Digital Filter (WDF) framework for modeling analog guitar effects pedals in real-time.

## Features

- **WDF Framework**: Wave Digital Filter implementation for analog circuit modeling
- **Real-time Audio**: Jack audio connection kit for low-latency processing
- **FX Building Blocks**: Resistors, capacitors, inductors, diodes, op-amps
- **Pedal Examples**: Overdrive, fuzz, and delay implementations

## Quick Start

```bash
# Clone the repo
git clone https://github.com/ajmwagar/pedalkernel
cd pedalkernel

# Build
cargo build --release

# Run with Jack
cargo run --example overdrive
```

## Architecture

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

## Examples

### Overdrive

```rust
use pedalkernel::{WdfTree, Resistor, Capacitor, Diode};

let mut pedal = Overdrive::new();
pedal.set_gain(0.7);
pedal.set_tone(0.5);
```

### Fuzz

```rust
use pedalkernel::pedals::FuzzFace;

let mut fuzz = FuzzFace::new();
fuzz.set_fuzz(0.9);
```

## License

MIT © Avery Wagar
