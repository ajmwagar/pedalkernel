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

PedalKernel is a circuit-to-audio compiler. You describe a pedal the way you'd draw it on a napkin -- resistors, caps, diodes, tubes, pots, with real values and real wiring -- and PedalKernel compiles it into a real-time audio engine using Wave Digital Filters.

The same `.pedal` file exports to KiCad for PCB layout and generates a Mouser bill of materials. Design the tone first, then solder it.

## What it sounds like

Every component shapes the sound the way the physical part would. A 220nF cap in the feedback loop rolls off differently than a 100nF. Germanium diodes clip softer than silicon. A 12AX7 saturates differently than a 12AU7. There are no "models" or "algorithms" to choose from -- just the circuit.

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

```
pedal "Fuzz Face" {
  components {
    C1: cap(2.2u)             # Big input cap = full bass into the fuzz
    R1: resistor(33k)
    R2: resistor(8.2k)
    Q1: pnp()                 # Germanium PNP pair
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

## Components

Everything you'd find on a pedal builder's bench -- and a tube amp chassis, and a synth rack:

### Passives

| Component | Syntax | What it does |
|-----------|--------|-------------|
| Resistor | `resistor(4.7k)` | Sets impedance, biasing, gain structure |
| Capacitor | `cap(220n)` | Frequency-dependent -- shapes the EQ curve |
| Inductor | `inductor(100m)` | Wah-style resonant peaks |
| Potentiometer | `pot(500k)` | Variable resistance, bound to knobs |
| Zener diode | `zener(4.7)` | Voltage clamp at a specific breakdown voltage |

### Diodes and clipping

| Component | Syntax | What it does |
|-----------|--------|-------------|
| Diode pair | `diode_pair(silicon\|germanium\|led)` | Symmetric clipping -- the core of overdrive |
| Single diode | `diode(silicon\|germanium\|led)` | Asymmetric clipping -- fuzz character |

### Transistors

| Component | Syntax | Variants |
|-----------|--------|----------|
| NPN BJT | `npn(2n3904)` | `2n3904`, `2n2222`, `bc108`, `bc109`, `2n5088`, `2n5089` |
| PNP BJT | `pnp(ac128)` | `2n3906`, `ac128`, `oc44`, `nkt275` (germanium) |
| N-JFET | `njfet(j201)` | `j201`, `2n5457`, `2n5952`, `2sk30a` (+ GR/Y/BL grades) |
| P-JFET | `pjfet(2n5460)` | `2n5460` |
| N-MOSFET | `nmos(2n7000)` | `2n7000`, `irf520` |
| P-MOSFET | `pmos(bs250)` | `bs250`, `irf9520` |
| Matched NPN | `matched_npn(ssm2210)` | `ssm2210`, `ca3046`, `lm394`, `that340` |
| Matched PNP | `matched_pnp(ssm2210)` | Same as matched NPN |

### Op-amps

| Component | Syntax | Character |
|-----------|--------|-----------|
| Op-amp | `opamp(tl072)` | `tl072` (clean, fast), `jrc4558` (warm, compressed), `lm308` (slow slew -- the RAT sound), `lm741`, `ne5532` (low noise), `rc4558`, `tl082`, `op07` |
| OTA | `opamp(ca3080)` | CA3080 operational transconductance amplifier |

### Vacuum tubes

| Component | Syntax | Character |
|-----------|--------|-----------|
| Triode | `triode(12ax7)` | `12ax7` (high gain), `12at7` (medium), `12au7` (low), `12ay7` (Fender tweed), `12bh7` (high current), `6386` (variable-mu) |
| Pentode | `pentode(el34)` | `ef86` (Vox preamp), `el84` (Vox output), `6l6gc`/`5881`/`kt66` (Fender Twin, Dumble), `el34`/`6ca7`/`kt77` (Marshall), `6550`/`kt88`/`kt90` (Ampeg SVT), `6aq5a`, `6973` |

The pentode Koren models capture the real differences between tube types -- 6L6GC's gradual Fender compression vs EL34's sharp Marshall breakup vs 6550's extended clean headroom.

### Delay and modulation

| Component | Syntax | What it does |
|-----------|--------|-------------|
| BBD | `bbd(mn3207)` | Bucket-brigade delay: `mn3207` (1024-stage, Boss CE-2 chorus), `mn3007` (low-noise, Boss DM-2), `mn3005` (4096-stage, Memory Man) |
| Delay line | `delay_line(1ms, 500ms, allpass, tape_oxide)` | Composable WDF delay with configurable range, interpolation (`linear`, `allpass`, `cubic`), and medium simulation |
| Tap | `tap(DL1, 0.75)` | Tap point on a delay line at a fractional position -- build multi-tap delays, ping-pong, rhythmic patterns |
| LFO | `lfo(sine, 10k, 100n)` | Modulation from physical RC timing. Waveforms: `sine`, `triangle`, `square`, `saw_up`, `saw_down`, `sample_and_hold` |
| Envelope follower | `envelope_follower(10k, 100n, 100k, 1u, 47k)` | Dynamics-reactive modulation from RC attack/release networks |

The delay line system models physical medium degradation -- magnetic tape oxide causes frequency-dependent high-frequency loss (like a real Roland RE-201), BBD charge leakage causes amplitude decay, and digital quantization models PT2399-style bit reduction. Medium presets: `re201`, `echoplex`, `lo_fi`.

### Opto and neon

| Component | Syntax | What it does |
|-----------|--------|-------------|
| Vactrol | `photocoupler(vtl5c3)` | Optical compression: `vtl5c3`, `vtl5c1`, `nsl32`, `t4b` (LA-2A) |
| Neon bulb | `neon(ne2)` | Vintage tremolo oscillator: `ne2`, `ne51`, `ne83` |

### Transformers and switching

| Component | Syntax | What it does |
|-----------|--------|-------------|
| Transformer | `transformer(15:1, 10H, ct_primary)` | Audio/output transformer with configurable turns ratio, inductance, and winding types (standard, center-tap, push-pull) |
| Switched cap | `cap_switched([100n, 220n, 470n])` | Capacitor with switchable values |
| Switched inductor | `inductor_switched([100m, 200m])` | Inductor with switchable values |
| Switched resistor | `resistor_switched([1k, 2.2k, 4.7k])` | Resistor with switchable values |
| Rotary switch | `rotary_switch([C_bright, C_normal, C_dark])` | Controls which switched component value is active |
| Switch | `switch(2)` | Simple SPST/SPDT mechanical switch |

### Synth ICs

| Component | Syntax | What it does |
|-----------|--------|-------------|
| VCO | `vco(cem3340)` | Voltage-controlled oscillator: `cem3340`, `as3340`, `v3340` |
| VCF | `vcf(cem3320)` | Voltage-controlled filter: `cem3320`, `as3320` |
| VCA | `vca(ssm2164)` | Voltage-controlled amplifier: `ssm2164`, `v2164` |
| Comparator | `comparator(lm311)` | `lm311`, `lm393` |
| Analog switch | `analog_switch(cd4066)` | `cd4066`, `dg411` |
| Tempco resistor | `tempco(1k, 3300)` | Temperature-compensating resistor for exponential converters |

### Engineering notation

`100p` = 100 pF, `220n` = 220 nF, `2.2u` = 2.2 uF, `100m` = 100 mH, `4.7k` = 4.7 kOhm, `1M` = 1 MOhm

### Wiring

Signal flow uses `->`. Comma-separated pins at a junction are electrically connected:

```
in -> C1.a              # Signal enters the coupling cap
C1.b -> R1.a, D1.a      # Cap output goes to both the resistor and diode
D1.b -> gnd              # Diode clips to ground
```

Reserved nodes: `in`, `out`, `gnd`, `vcc`

### Controls

Map physical pot positions to named knobs:

```
Gain.position -> "Drive" [0.0, 1.0] = 0.5
```

---

## UI manifests

Pedal UI manifests (JSON) live under `pedalkernel-vst/ui/`.

- Schema: `pedalkernel-vst/ui/ui_manifest.schema.json`
- Free pedal manifests: `pedalkernel-vst/ui/manifests/free/`

These are non-secret control-surface descriptions (controls list, enclosure preset, optional style hints) used by UI/skin generation in downstream tooling.

See: `docs/UI_MANIFESTS.md`

## Quick start

```bash
git clone https://github.com/ajmwagar/pedalkernel
cd pedalkernel
cargo build --release
cargo test

# Render to WAV -- listen to the circuit
cargo run --example overdrive
cargo run --example fuzzface

# Parse any .pedal file
cargo run --example parse_pedal -- examples/pedals/fuzz/big_muff.pedal

# Export KiCad netlist for PCB layout
cargo run --example parse_pedal -- examples/pedals/overdrive/klon_centaur.pedal

# Compare diode clipping: silicon vs germanium vs LED
cargo run --example direct_wdf

# Interactive TUI -- tweak knobs in real time
cargo run --example tui --features tui -- examples/pedals/overdrive/tube_screamer.pedal
```

---

## From tone to PCB

The same `.pedal` file drives three outputs:

```
                          +---> WDF audio engine ---> WAV / JACK real-time
                          |
  .pedal file ---> parse -+---> KiCad netlist ---> PCB layout
                          |
                          +---> Bill of Materials ---> Mouser order
```

> Note on UI/skins:
> - `.pedal` is **circuit-only** (signal path + parameter mapping).
> - `.pedalhw` is **hardware/skin metadata** (e.g., parts specs, faceplate/finish in the pro toolchain).
> - This repo also contains **UI manifests** (JSON) under `pedalkernel-vst/ui/` for non-secret control-surface metadata (labels, knob/switch list, enclosure preset).
> - The pro pipeline may merge `.pedalhw` + UI manifest + internal assets when generating final branded skins.

### KiCad export

Every component maps to a real KiCad symbol. Triodes get `Valve:ECC83`/`ECC81`/`ECC82`. Power pentodes get `Valve:6L6GC`/`Valve:EL34`/`Valve:6550`. JFETs get `Device:Q_NJFET_DGS`. Transformers get `Transformer:Transformer_1P_CT_1S_CT`. Nets are preserved exactly. Open the `.net` file in KiCad and start laying out copper.

### Bill of materials

The BOM engine maps your circuit to real parts from a curated database -- Yageo metal film resistors, WIMA film caps, Nichicon electrolytics, Alpha pots, JJ Electronic tubes. Upload the CSV directly to Mouser:

```rust
let bom = pedalkernel::hw::build_bom(&pedal, None);
print!("{}", pedalkernel::hw::format_bom_table(&pedal.name, &bom, 1));
// -> Mouser P/Ns, quantities, descriptions, ready to order
```

### Hardware specs (`.pedalhw` files)

For builders who need to know if a part will survive the voltage. Declare real specs alongside your circuit:

```
# fuzz_face.pedalhw
Q1: vce_max(32) part("AC128")
Q2: vce_max(32) part("AC128")
C2: voltage_rating(16)
```

Run voltage compatibility checks before you power anything up:

```rust
let warnings = pedalkernel::hw::check_voltage_with_specs(&pedal, 18.0, &limits);
// [Danger] Q1: Germanium transistor exceeds Vce(max) 32V at 18V
// [Info]   V1: Tube needs 150-400V plate supply; at 18V the WDF model
//              runs fine but a physical build needs a B+ supply
```

Without a `.pedalhw` file, heuristic checks still catch obvious problems -- germanium transistors in fuzz circuits above 12V, undersized electrolytic caps, tubes at pedal voltages.

---

## TUI control surface

Tweak any `.pedal` file as an interactive ASCII pedalboard:

```bash
cargo run --example tui --features tui -- examples/pedals/overdrive/tube_screamer.pedal
```

```
+--------------------------------------+
|          TUBE SCREAMER               |
|                                      |
|      .---.          .---.           |
|     /  |  \        /     \          |
|    |   |   |      |     \ |          |
|     \     /        \     /          |
|      '---'          '---'           |
|    >>Drive<<         Level           |
|       0.50           0.80            |
|                                      |
|            .----------.              |
|            |  ACTIVE  |              |
|            '----------'              |
|                                      |
|  <-> adjust  ^v fine  Tab next  Q quit|
+--------------------------------------+
```

Works with any pedal file. `big_muff.pedal` gives you three knobs. The control surface shapes to whatever you wire up.

---

## Supply voltage

Pedals run at 9V by default. Crank it to 12V or 18V for more headroom -- the WDF engine models how the active elements respond to higher rail voltage. The clipping threshold shifts, the gain stages swing further before saturating, the whole feel opens up. Just like plugging a real Tube Screamer into an 18V adapter.

For tube amps, supply voltage goes up to 500V. Set `supply 460V` in a `.pedal` file and the pentode plate curves, triode operating points, and Newton-Raphson bounds all adjust to match real B+ rails.

```rust
proc.set_supply_voltage(12.0);   // More headroom, cleaner clipping
proc.set_supply_voltage(18.0);   // Even more -- like a Voodoo Lab Pedal Power
proc.set_supply_voltage(460.0);  // Fender Twin Reverb B+ rail
```

---

## How it works

The compiler transforms your circuit into a Wave Digital Filter tree:

1. **Circuit graph** -- Union-Find merges connected pins into nodes. Components become edges.

2. **Nonlinear root identification** -- BFS from the input finds diodes, JFETs, triodes, pentodes, and BJTs. Each becomes a WDF root. Its neighboring passives form the WDF tree.

3. **Series-parallel decomposition** -- The passive subgraph around each nonlinear element reduces into a binary tree of Series and Parallel adaptors.

4. **Impedance balancing** -- A raw voltage source (Rp=1) in parallel with a 100k resistor would attenuate the signal to nothing. The compiler adjusts Vs port resistance to match its sibling, giving balanced gamma for efficient signal transfer.

5. **Per-sample processing** -- Four phases, zero allocation:
   - Scatter up: leaves to root (reflected waves propagate)
   - Root solve: Newton-Raphson on the nonlinear element (Shockley equation for diodes, Koren equation for triodes/pentodes)
   - Scatter down: root to leaves (incident waves propagate)
   - State update: capacitors and inductors latch

### Compiled Big Muff topology

Two cascaded clipping stages. Each diode pair gets its own WDF tree:

```
Stage 1:                          Stage 2:
   [DiodePair D1]                    [DiodePair D2]
         |                                 |
   ParallelAdaptor                   ParallelAdaptor
    /          \                      /          \
  Series      R1(10k)              Series       R2(47k)
  /     \                          /     \
Vs    C1(100n)                   Vs    C2(100n)
```

---

## Performance

All pedals run comfortably in real-time. Benchmarks on an Apple M-series chip at 48 kHz:

| Pedal | CPU Budget | Samples/sec | Realtime × |
|-------|------------|-------------|------------|
| Tube Screamer | 1.2% | 3.8M | 80× |
| Big Muff | 1.7% | 2.8M | 59× |
| Fuzz Face | 0.04% | 122M | 2500× |
| Blues Driver | 5.9% | 818K | 17× |
| Dyna Comp | 0.4% | 12M | 252× |
| Klon Centaur | 1.4% | 3.5M | 72× |
| ProCo RAT | 0.3% | 16M | 330× |
| Boss CE-2 (BBD) | 0.5% | 9.9M | 207× |

**CPU Budget** = fraction of real-time budget consumed. Under 100% means glitch-free audio.

### FLOPS breakdown

WDF processing cost per sample depends on circuit complexity:

| Element | Est. FLOPs |
|---------|------------|
| Series adaptor | 9 |
| Parallel adaptor | 10 |
| Capacitor | 2 |
| Diode Newton solve | 220 (55 × 4 iterations) |
| **Typical clipper stage** | **~250 total** |

A Tube Screamer (1 clipping stage) needs ~12 MFLOPS at 48 kHz. A Big Muff (4 cascaded stages) needs ~48 MFLOPS.

The BBD model (Boss CE-2) includes full analog bucket-brigade emulation:
- **Companding** — NE571-style 2:1 compression/expansion with mismatched attack/release (causes "breathing")
- **Charge leakage** — per-stage loss accumulates across 1024+ stages, darkening long delays (the Memory Man warmth)
- **Clock feedthrough** — parasitic capacitance couples switching clock into signal (the characteristic BBD whine)
- **Bandwidth limiting** — Nyquist limit at half clock frequency with anti-alias LPF
- **Soft clipping** — BBD voltage swing saturation with cubic waveshaping

Despite this, BBD overhead is modest (~0.5% CPU) because most operations are simple one-pole filters.

### Scalability

Cascaded WDF stages scale linearly:

| Stages | ns/sample | CPU @ 48kHz |
|--------|-----------|-------------|
| 1 | 173 | 0.8% |
| 2 | 268 | 1.3% |
| 4 | 592 | 2.8% |
| 8 | 1249 | 6.0% |

### Sample rate scaling

The Tube Screamer at various sample rates:

| Sample Rate | Realtime × |
|-------------|------------|
| 44.1 kHz | 90× |
| 48 kHz | 82× |
| 96 kHz | 42× |
| 192 kHz | 21× |

Run benchmarks yourself:

```bash
cargo bench --bench wdf_bench
```

---

## API

```rust
use pedalkernel::{dsl, compiler, kicad};

// Parse and compile
let pedal = dsl::parse_pedal_file(&src).unwrap();
let mut proc = compiler::compile_pedal(&pedal, 48000.0).unwrap();

// Play
proc.set_control("Drive", 0.7);
let output = proc.process(input_sample);

// Export for PCB
let netlist = kicad::export_kicad_netlist(&pedal);

// Or use the built-in overdrive directly
let mut od = pedalkernel::pedals::Overdrive::new(48000.0);
od.set_gain(0.7);
let output = od.process(input_sample);
```

---

## Feature flags

| Feature | Default | What it adds |
|---------|---------|-------------|
| `jack-rt` | Yes | JACK real-time audio -- sub-5ms latency through a Scarlett 2i2 |
| `tui` | Yes | Interactive ASCII control surface via `ratatui` |
| `cli` | Yes | Command-line interface via `clap` |
| `hardware` | No | BOM generation, voltage checks, `.pedalhw` specs |

Minimal DSP-only build:

```bash
cargo build --no-default-features
```

---

## Included pedals, amps & synths

`.pedal` files are organized under `examples/`:

**Pedals** (`examples/pedals/`)

| Circuit | Category | Character |
|---------|----------|-----------|
| Tube Screamer | overdrive | Mid-hump overdrive, the sound of blues rock |
| Blues Driver | overdrive | Dynamic, touch-sensitive overdrive |
| Klon Centaur | overdrive | Transparent overdrive, clean/dirty mix |
| Fulltone OCD | overdrive | MOSFET clipping, amp-like breakup |
| Fuzz Face | fuzz | Velcro-rip fuzz, NPN/PNP polarity flip |
| Big Muff | fuzz | Sustained wall of fuzz, Smashing Pumpkins territory |
| ProCo RAT | distortion | Tight distortion, filter sweep |
| MXR Phase 90 | phaser | 4-stage JFET phaser, swept comb filtering |
| Boss CE-2 | modulation | BBD analog chorus |
| Dyna Comp | compressor | Optical compression, country squish |

**Amps** (`examples/amps/`)

| Circuit | Tubes | Character |
|---------|-------|-----------|
| Tweed Deluxe 5E3 | 12AY7 + 12AX7 | Touch-sensitive breakup, Neil Young |
| Bassman 5F6-A | 12AY7 + 12AX7 | TMB tone stack, the grandfather of Marshall |
| Marshall JTM45 | ECC83 x3 | British crunch, Clapton "Beano" tone |

Power pentode support for 6L6GC, EL34, and 6550 is now in the engine -- Fender Twin Reverb, Marshall JCM800, and Dumble ODS amp models are next.

**Synth modules** (`examples/synths/`)

| Circuit | ICs | Character |
|---------|-----|-----------|
| CEM3340 VCO | CEM3340 | Classic Curtis sawtooth/pulse/triangle oscillator |
| Moog Ladder VCF | Matched NPN pairs | 4-pole transistor ladder lowpass |
| Minisynth | CEM3340 + CEM3320 + SSM2164 | Complete mono voice: VCO + VCF + VCA |

---

## Python tools

Optional standalone scripts in `tools/`:

```bash
python3 -m venv tools/.venv
tools/.venv/bin/pip install -r tools/requirements.txt

# Render EE-style schematics from .pedal files
tools/.venv/bin/python tools/schematic.py examples/pedals/overdrive/tube_screamer.pedal -o ts.png

# Generate Mouser BOM
tools/.venv/bin/python tools/mouser_bom.py examples/pedals/fuzz/big_muff.pedal --qty 5 --csv bom.csv
```

---

## License

AGPLv3 (c) [Avery Wagar](https://github.com/ajmwagar)
