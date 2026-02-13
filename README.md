<div align="center">
  <img src="logo/pedalkernel-logo.png" alt="PedalKernel Logo" width="200">
  <h1>PedalKernel</h1>
  <p><strong>Compile guitar pedal circuits into real-time DSP kernels</strong></p>
  <p>Write <code>.pedal</code> files with real component values. Get WDF audio engines and KiCad netlists.</p>

  [![Rust](https://img.shields.io/badge/rust-%23000000.svg?style=for-the-badge&logo=rust&logoColor=white)](https://www.rust-lang.org/)
  [![JACK](https://img.shields.io/badge/JACK-Audio-ff6b6b?style=for-the-badge&logo=audio-technica&logoColor=white)](https://jackaudio.org/)
  [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg?style=for-the-badge)](https://opensource.org/licenses/MIT)
</div>

---

## The `.pedal` DSL

Define circuits the way you'd read a schematic. Components use real values with engineering notation, nets describe signal flow, and controls map knobs to parameters.

### Tube Screamer

```
# Tube Screamer TS-808 style overdrive
# Simplified clipping stage: input cap -> feedback network with diode pair

pedal "Tube Screamer" {
  components {
    R1: resistor(4.7k)
    C1: cap(220n)
    D1: diode_pair(silicon)
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

### Fuzz Face

```
# Fuzz Face style germanium fuzz
# Two-transistor topology with input cap and biasing

pedal "Fuzz Face" {
  components {
    C1: cap(2.2u)
    R1: resistor(33k)
    R2: resistor(8.2k)
    Q1: pnp()
    Q2: pnp()
    C2: cap(10u)
    R3: resistor(100k)
    Fuzz: pot(1k)
    Volume: pot(500k)
  }
  nets {
    in -> C1.a
    C1.b -> Q1.base
    vcc -> R1.a
    R1.b -> Q1.collector, R2.a
    R2.b -> Q2.base
    Q1.emitter -> Fuzz.a
    Fuzz.b -> gnd
    Q2.collector -> R3.a
    R3.b -> gnd
    Q2.emitter -> vcc
    Q2.collector -> C2.a
    C2.b -> Volume.a
    Volume.b -> out
  }
  controls {
    Fuzz.position -> "Fuzz" [0.0, 1.0] = 0.7
    Volume.position -> "Volume" [0.0, 1.0] = 0.5
  }
}
```

### Big Muff Pi

```
# Big Muff Pi style distortion
# Four-stage clipping with tone stack

pedal "Big Muff" {
  components {
    C1: cap(100n)
    R1: resistor(10k)
    D1: diode_pair(silicon)
    R2: resistor(47k)
    C2: cap(100n)
    D2: diode_pair(silicon)
    R3: resistor(100k)
    C3: cap(4n)
    R4: resistor(22k)
    C4: cap(10n)
    Tone: pot(100k)
    Sustain: pot(100k)
    Volume: pot(100k)
  }
  nets {
    in -> C1.a
    C1.b -> R1.a, D1.a
    D1.b -> gnd
    R1.b -> C2.a
    C2.b -> R2.a, D2.a
    D2.b -> gnd
    R2.b -> R3.a
    R3.b -> Tone.a
    Tone.b -> C3.a, C4.a
    C3.b -> R4.a
    C4.b -> R4.a
    R4.b -> Volume.a
    Volume.b -> out
  }
  controls {
    Sustain.position -> "Sustain" [0.0, 1.0] = 0.7
    Tone.position -> "Tone" [0.0, 1.0] = 0.5
    Volume.position -> "Volume" [0.0, 1.0] = 0.5
  }
}
```

---

## DSL Reference

### Components

| Syntax | Description |
|--------|-------------|
| `resistor(4.7k)` | Resistor |
| `cap(220n)` | Capacitor |
| `inductor(100m)` | Inductor |
| `diode_pair(silicon\|germanium\|led)` | Anti-parallel diode pair |
| `diode(silicon\|germanium\|led)` | Single diode |
| `pot(500k)` | Potentiometer |
| `npn()` / `pnp()` | BJT transistors |
| `opamp()` | Operational amplifier |

### Engineering Notation

| Suffix | Multiplier | Example |
|--------|-----------|---------|
| `p` | 10^-12 (pico) | `100p` = 100 pF |
| `n` | 10^-9 (nano) | `220n` = 220 nF |
| `u` | 10^-6 (micro) | `2.2u` = 2.2 uF |
| `m` | 10^-3 (milli) | `100m` = 100 mH |
| `k` | 10^3 (kilo) | `4.7k` = 4.7 kOhm |
| `M` | 10^6 (mega) | `1M` = 1 MOhm |

### Nets

Signal flow uses `->` with comma-separated nodes at the same junction:

```
in -> C1.a              # reserved node to component pin
C1.b -> R1.a, D1.a      # one-to-many junction
D1.b -> gnd              # component pin to ground
```

### Reserved Nodes

| Node | Purpose |
|------|---------|
| `in` | Signal input |
| `out` | Signal output |
| `gnd` | Ground reference |
| `vcc` | Power supply |

### Controls

Map component properties to named knobs with range and default:

```
Gain.position -> "Drive" [0.0, 1.0] = 0.5
```

### Comments

Lines starting with `#` are comments.

---

## Quick Start

```bash
git clone https://github.com/ajmwagar/pedalkernel
cd pedalkernel

cargo build --release
cargo test

# Render overdrive/fuzz to WAV files
cargo run --example overdrive
cargo run --example fuzzface

# Parse a .pedal file and export KiCad netlist
cargo run --example parse_pedal
cargo run --example parse_pedal -- examples/big_muff.pedal

# Compare silicon/germanium/LED diode models
cargo run --example direct_wdf

# Interactive TUI pedal control surface
cargo run --example tui --features tui -- examples/tube_screamer.pedal
```

---

## Features

- **DSL Parser** — `nom`-based parser for `.pedal` files
- **WDF Engine** — series/parallel adaptors with verified scattering matrices, diode pair and single-diode nonlinear roots via Newton-Raphson
- **Zero-Allocation Hot Path** — per-sample processing with no heap allocation
- **KiCad Export** — generate netlists from the same `.pedal` file for PCB prototyping
- **WAV Output** — offline rendering via `hound` for tone prototyping without a running audio server
- **Pedal Library** — ready-to-use Overdrive, Fuzz Face, and Delay implementations
- **TUI Control Surface** — interactive ASCII pedal with rotary knobs via `ratatui` (optional `tui` feature)

### Built-in Pedals

| Pedal | Description | Status |
|-------|-------------|--------|
| **Overdrive** | Tube Screamer-style WDF diode-pair clipping | Working |
| **FuzzFace** | Germanium single-diode asymmetric clipping | Working |
| **Delay** | Digital delay line with feedback | Working |

---

## TUI Control Surface

Visualize and tweak any `.pedal` file as an interactive ASCII pedal — like staring down at a real pedalboard in your terminal.

```bash
cargo run --example tui --features tui -- examples/tube_screamer.pedal
```

```
╔══════════════════════════════════════╗
║          TUBE SCREAMER               ║
║                                      ║
║      ╭───╮          ╭───╮           ║
║     ╱  |  ╲        ╱     ╲          ║
║    │   |   │      │     \ │          ║
║     ╲     ╱        ╲     ╱          ║
║      ╰───╯          ╰───╯           ║
║    >>Drive<<         Level           ║
║       0.50           0.80            ║
║                                      ║
║            ╭──────────╮              ║
║            │  ACTIVE  │              ║
║            ╰──────────╯              ║
║                                      ║
║  ←→ adjust  ↑↓ fine  Tab next  Q quit║
╚══════════════════════════════════════╝
```

**Controls:** `←→` coarse adjust, `↑↓` fine adjust, `Tab`/`Shift+Tab` cycle knobs, `R` reset to default, `Space` toggle bypass, `Q` quit.

Works with any pedal file — try `big_muff.pedal` for a 3-knob layout.

<!-- TODO: Add asciinema recording or VHS gif (https://github.com/charmbracelet/vhs) -->

---

## Architecture

```
 .pedal file
     |
     v
 +----------+     +--------------+
 |DSL Parser|---->| KiCad Export  |---> .net file (PCB layout)
 +----+-----+     +--------------+
      |
      v
 +----------------------------------+
 |          WDF Engine              |
 |                                  |
 |  Leaves: R, C, L, VoltageSource  |
 |  Adaptors: Series, Parallel      |
 |  Roots: DiodePair, Diode (NR<=8) |
 +------------+---------------------+
              |
              v
 +----------------------------------+
 |        Pedal Library             |
 |  Overdrive | FuzzFace | Delay    |---> WAV output (hound)
 +----------------------------------+
```

### WDF Processing Pipeline

Each sample is processed in four phases with zero heap allocation:

```
1. scatter_up    Leaves -> Root    Reflected waves propagate bottom-up
2. root_solve    Nonlinear root   Newton-Raphson on Shockley equation
3. scatter_down  Root -> Leaves    Incident waves propagate top-down
4. state_update  Reactive elems   Capacitors/inductors latch new state
```

### Clipper Tree Topology

```
       [DiodePair root]        <- Newton-Raphson solver
             |
        SeriesAdaptor          <- Rp = R1 + R2
         /         \
  VoltageSource   ParallelAdaptor  <- Rp = R1*R2/(R1+R2)
   (input)         /          \
               Resistor    Capacitor
```

---

## Rust API

```rust
// Parse a .pedal file
let pedal = pedalkernel::dsl::parse_pedal_file(&src).unwrap();

// Export KiCad netlist
let netlist = pedalkernel::kicad::export_kicad_netlist(&pedal);

// Use the WDF overdrive
let mut od = pedalkernel::pedals::Overdrive::new(48000.0);
od.set_gain(0.7);
let output = od.process(input_sample);

// Render to WAV
pedalkernel::wav::render_to_wav(&mut od, &input, Path::new("out.wav"), 48000).unwrap();

// Run through JACK for real-time audio (cargo build --features jack-rt)
// let od = pedalkernel::pedals::Overdrive::new(48000.0);
// let _client = pedalkernel::AudioEngine::run("PedalKernel", od).unwrap();
```

---

## Benchmarks

```bash
cargo bench
```

---

## License

MIT © [Avery Wagar](https://github.com/ajmwagar)
