# PedalKernel DSL Reference

PedalKernel uses two file formats: `.pedal` files define individual guitar pedal circuits, and `.board` files chain multiple pedals into a pedalboard.

## `.pedal` File Format

A `.pedal` file describes a guitar pedal's circuit topology as a netlist. The compiler transforms this into a real-time Wave Digital Filter (WDF) audio processor.

### Structure

```
pedal "<Name>" {
  supply <voltage>V        # optional — defaults to 9V if omitted
  components {
    <component declarations>
  }
  nets {
    <net connections>
  }
  controls {
    <control mappings>
  }
}
```

All three sections are required (though `controls` may be omitted if the pedal has no knobs). Comments use `#` and extend to end of line.

### Supply Voltage

The `supply` keyword specifies the circuit's DC supply voltage. This affects the headroom for clipping/saturation modeling and should match the actual power supply of the equipment being modeled:

| Equipment Type | Typical Supply | Example |
|----------------|----------------|---------|
| Guitar pedals | 9V | `supply 9V` |
| 12V modded pedals | 12V | `supply 12V` |
| Transistor preamps | 24-30V | `supply 24V` |
| Tube preamps | 250-400V | `supply 285V` |

If omitted, the supply defaults to **9V** (standard guitar pedal voltage).

```
# Tube amplifier preamp
pedal "Tweed Deluxe" {
  supply 350V    # B+ for tube stages
  ...
}

# Standard guitar pedal (9V is default, but explicit is clearer)
pedal "Tube Screamer" {
  supply 9V
  ...
}
```

### Voltage Sag

Voltage sag is the dynamic drop in B+ voltage when tubes draw current under load. The WDF engine models this automatically based on circuit topology:

1. Loud signal → tubes conduct more → increased plate current
2. Current flows through power supply impedance
3. Voltage drop: ΔV = I × Z_supply
4. Lower plate voltage → reduced gain → natural compression
5. Signal quiets → voltage recovers (slowly)

Sag is more pronounced with:
- High-impedance supplies (tube rectifiers, small transformers)
- Cathode-biased output stages
- Power tubes (6L6, EL34) vs preamp tubes (12AX7)

Sag creates the "breathing" feel of vintage tube amps and contributes to dynamic compression in studio compressors like the Fairchild 670.

### Components

Each component is declared as `<id>: <type>(<params>)`.

| Type | Syntax | Description |
|------|--------|-------------|
| Resistor | `resistor(<value>)` | Fixed resistor |
| Capacitor | `cap(<value>)` | Capacitor |
| Inductor | `inductor(<value>)` | Inductor |
| Potentiometer | `pot(<value>)` | Variable resistor (controllable knob) |
| Diode pair | `diode_pair(<type>)` | Symmetric clipping diode pair |
| Single diode | `diode(<type>)` | Asymmetric single diode |
| Zener diode | `zener(<voltage>)` | Zener diode with breakdown voltage (e.g., `zener(5.1)`) |
| NPN transistor | `npn()` | NPN BJT (modeled as gain stage) |
| PNP transistor | `pnp()` | PNP BJT (modeled as gain stage) |
| Op-amp | `opamp()` | Operational amplifier |
| N-channel JFET | `njfet(<model>)` | N-channel JFET (nonlinear WDF root) |
| P-channel JFET | `pjfet(<model>)` | P-channel JFET (nonlinear WDF root) |
| Triode | `triode(<type>)` | Vacuum tube triode (see below) |
| Pentode | `pentode(<type>)` | Vacuum tube pentode (see below) |
| Photocoupler | `photocoupler(<model>)` | Vactrol / optocoupler (controlled resistance) |
| Neon bulb | `neon()` or `neon(<model>)` | Neon lamp for relaxation oscillators |
| BBD | `bbd(<model>)` | Bucket-brigade device delay line |
| Delay line | `delay_line(<min>, <max> [, <interp>] [, medium: <medium>])` | Generic ring-buffer delay |
| Tap | `tap(<delay_id>, <ratio>)` | Read-only tap into a delay line |
| LFO | `lfo(<waveform>, <R>, <C>)` | LFO with RC timing (f = 1/2piRC) |
| Envelope Follower | `envelope_follower(<atk_R>, <atk_C>, <rel_R>, <rel_C>, <sens_R>)` | Envelope detector with RC timing |

**Engineering notation** is supported for component values:

| Suffix | Multiplier | Example |
|--------|-----------|---------|
| `p` | 10^-12 (pico) | `100p` = 100 pF |
| `n` | 10^-9 (nano) | `47n` = 47 nF |
| `u` | 10^-6 (micro) | `10u` = 10 uF |
| `m` | 10^-3 (milli) | `100m` = 100 mH |
| `k` | 10^3 (kilo) | `4.7k` = 4.7 kOhm |
| `M` | 10^6 (mega) | `1M` = 1 MOhm |

**Diode types**: `silicon`, `germanium`, `led`

**Zener voltages**: Common values are 3.3, 4.7, 5.1, 5.6, 6.2, 9.1, 12 (in volts)

**JFET models**: `j201`, `2n5457`, `2n5460`, `2sk30` (or `2sk30a`, `2sk30-gr`, `2sk30-y`, `2sk30-bl` for graded variants)

**Triode types**: `12ax7`, `12at7`, `12au7`, `12bh7`, `12ay7`

| Type   | µ (mu) | Use case                              |
|--------|--------|---------------------------------------|
| 12AX7  | 100    | High-gain preamp, voltage amplifier   |
| 12AT7  | 60     | Medium gain, phase inverter, driver   |
| 12AU7  | 17     | Low gain, cathode follower            |
| 12BH7  | 17     | High-current cathode follower         |
| 12AY7  | 44     | Lower-gain preamp (cleaner)           |

Triode pins: `.grid`, `.plate`, `.cathode`

**Pentode types**: `ef86`, `el84`, `el34`, `6l6gc`, `6v6`, `kt66`, `6aq5a`

| Type   | Max Watts | Use case                              |
|--------|-----------|---------------------------------------|
| EF86   | 1.2       | Low-noise preamp (Vox, Ampeg)         |
| EL84   | 12        | Small power amp (Champ, AC15)         |
| EL34   | 25        | Power amp (Marshall, Hiwatt)          |
| 6L6GC  | 30        | Power amp (Bassman, Twin)             |
| 6V6    | 14        | Medium power amp (Tweed Deluxe)       |
| KT66   | 35        | Power amp (early Marshall, Quad)      |
| 6AQ5A  | 12        | Output stage, small amps              |

Pentode pins: `.grid`, `.plate`, `.cathode`, `.screen`

The screen grid requires a dropping resistor from B+ and bypass cap to ground:
```
vcc -> R_screen.a
R_screen.b -> V1.screen, C_screen.a
C_screen.b -> gnd
```

**Photocoupler models**: `vtl5c3`, `vtl5c1`, `nsl32`

**Neon bulb models**: `ne2` (default), `ne51`, `ne83`
- NE-2: 90V striking, 60V maintaining — standard miniature neon
- NE-51: 95V striking, 65V maintaining — higher current
- NE-83: 65V striking, 50V maintaining — lower voltage variant

Neon bulbs are used in vintage tremolo circuits (Fender Vibrato, Wurlitzer) as part of relaxation oscillators. When paired with an LDR (photocoupler), they create smooth optical modulation.

**LFO waveforms**: `sine`, `triangle`, `square`, `saw_up`, `saw_down`, `sample_and_hold`

### Nets

Nets describe how component pins connect. Each net is `<from> -> <to1>, <to2>, ...`.

**Reserved nodes**:
- `in` — audio input
- `out` — audio output
- `gnd` — ground reference
- `vcc` — power supply

**Component pins** use dot notation: `C1.a`, `R1.b`, `D1.a`, `Q1.base`.

Two-terminal components (resistors, capacitors, inductors, diodes) have pins `a` and `b`. Potentiometers have pins `a` and `b` (2-terminal variable resistor), or `a`, `w` (wiper), and `b` (3-terminal pot for crossfade/blend). Transistors have `base`, `collector`, `emitter`. JFETs have `gate`, `drain`, `source`. Op-amps have `pos`, `neg`, `out`. LFOs and envelope followers have `out` (modulation output) which connects to modulation inputs like `<jfet>.vgs` or `<photocoupler>.led`.

**3-terminal pots:** When a pot uses the `.w` (wiper) pin in nets, it is modeled as two linked variable resistors sharing the wiper node. As position increases, R(a→w) increases and R(w→b) decreases, maintaining R(a→w) + R(w→b) = max_R. This enables crossfade/blend topologies where signals connect to both `.a` and `.b` with the output taken from `.w`.

```
in -> C1.a                       # input to capacitor pin a
C1.b -> R1.a, D1.a              # capacitor pin b fans out to R1 and D1
D1.b -> gnd                     # diode to ground
R1.b -> out                     # resistor to output
```

### Controls

Controls map component properties to named knobs with ranges and defaults:

```
<component>.<property> -> "<Label>" [<min>, <max>] = <default>
```

The `position` property is used for potentiometers (0.0 = fully CCW, 1.0 = fully CW):

```
Drive.position -> "Drive" [0.0, 1.0] = 0.5
Level.position -> "Level" [0.0, 1.0] = 0.8
```

### Modulation Components

**LFO** — Specify waveform and physical RC timing components. Frequency is derived from the RC constant: `f = 1/(2*pi*R*C)`. The R and C values appear in the KiCad netlist and Mouser BOM as real components.

```
LFO1: lfo(triangle, 100k, 220n)  # f ≈ 7.2 Hz triangle
LFO1.out -> OC1.led              # drive photocoupler LED
```

**Envelope Follower** — Specify attack/release RC timing and sensitivity resistor. All 5 component values are physical parts that appear in KiCad export and BOM.

```
# envelope_follower(attack_R, attack_C, release_R, release_C, sensitivity_R)
EF1: envelope_follower(1k, 4.7u, 100k, 1u, 20k)
# Attack τ = 1kΩ × 4.7µF = 4.7ms (fast attack)
# Release τ = 100kΩ × 1µF = 100ms (smooth release)
# Sensitivity = 20kΩ / 10kΩ = 2.0x gain
EF1.out -> J1.vgs                # modulate JFET gate voltage
```

### Delay Components

**BBD (Bucket-Brigade Device)** — Models analog bucket-brigade delay ICs with compander, charge leakage, clock feedthrough, and noise. Internally uses a `DelayLine` for buffer management.

```
BBD1: bbd(mn3207)                # MN3207 (512 stages, typical chorus/flanger)
BBD1: bbd(mn3007)                # MN3007 (1024 stages, longer delays)
BBD1: bbd(mn3005)                # MN3005 (4096 stages, up to ~300ms)
```

**BBD models**: `mn3207`, `mn3007`, `mn3005`

BBD pins: `in` (audio input), `out` (audio output), `clock` (modulation target for LFO).

**Delay Line** — Generic ring-buffer delay with configurable interpolation and optional physical medium simulation. Splits the WDF tree at its boundaries: the write side accepts signal from one subtree, the read side feeds another.

```
# Basic (defaults: allpass interpolation, no medium)
DL1: delay_line(1ms, 1200ms)

# Explicit interpolation mode
DL1: delay_line(1ms, 1200ms, allpass)

# With physical medium simulation
DL1: delay_line(80ms, 1200ms, medium: tape_oxide)

# Both interpolation and medium
DL1: delay_line(80ms, 1200ms, allpass, medium: bbd_leakage)
```

**Interpolation modes**: `linear`, `allpass` (default — Thiran approximation, zipper-free), `cubic` (Hermite)

**Medium types**:
| Medium | Description |
|--------|-------------|
| `none` | Clean digital buffer (default) |
| `tape_oxide` | Tape self-demagnetization — progressive HF loss with distance |
| `bbd_leakage` | BBD charge leakage — amplitude decay across stages |
| `digital_quantize` | Bit-depth reduction (placeholder) |

Delay line pins: `delay_time` (control target), `feedback` (control target), `speed_mod` (modulation target for LFO wow/flutter).

**Tap** — Read-only output port into a parent delay line at a fixed ratio of the base delay time. Multiple taps model multi-head tape machines (e.g., RE-201 heads at 1×, 2×, 4× base delay).

```
TAP1: tap(DL1, 1.0)    # reads at 1× base delay
TAP2: tap(DL1, 2.0)    # reads at 2× base delay
TAP3: tap(DL1, 4.0)    # reads at 4× base delay
```

**Delay + Tap example (RE-201 Space Echo):**

```
pedal "Space Echo" {
  components {
    DL1: delay_line(80ms, 1200ms, allpass, medium: tape_oxide)
    TAP1: tap(DL1, 1.0)     # Head 1 (3cm from write)
    TAP2: tap(DL1, 2.0)     # Head 2 (6cm from write)
    TAP3: tap(DL1, 4.0)     # Head 3 (12cm from write)
    LFO1: lfo(sine, 680k, 1u)  # wow/flutter
  }
  nets {
    in -> DL1.in
    TAP1.out -> out
    TAP2.out -> out
    TAP3.out -> out
    LFO1.out -> DL1.speed_mod
  }
  controls {
    DL1.delay_time -> "Repeat Rate" [0.0, 1.0] = 0.5
    DL1.feedback -> "Intensity" [0.0, 1.0] = 0.3
  }
}
```

### Complete Example

```
# Tube Screamer TS-808 clipping stage
pedal "Tube Screamer" {
  components {
    Drive: pot(4.7k)         # variable feedback resistor
    C1: cap(47n)             # feedback capacitor
    D1: diode_pair(silicon)  # symmetric silicon clipping diodes
    Level: pot(100k)         # output volume
  }
  nets {
    in -> Drive.a, C1.a
    Drive.b -> C1.b, D1.a, Level.a
    D1.b -> gnd
    Level.b -> out
  }
  controls {
    Drive.position -> "Drive" [0.0, 1.0] = 0.5
    Level.position -> "Level" [0.0, 1.0] = 0.8
  }
}
```

### Included Pedals

| File | Pedal | Controls |
|------|-------|----------|
| `tube_screamer.pedal` | Tube Screamer TS-808 | Drive, Level |
| `fuzz_face.pedal` | Fuzz Face | Fuzz, Volume |
| `big_muff.pedal` | Big Muff | Sustain, Tone, Volume |
| `dyna_comp.pedal` | MXR Dyna Comp | Sensitivity, Output |
| `proco_rat.pedal` | ProCo RAT | Distortion, Filter, Volume |
| `blues_driver.pedal` | Boss Blues Driver | Gain, Tone, Level |
| `klon_centaur.pedal` | Klon Centaur | Gain, Treble, Output |

---

## `.board` File Format

A `.board` file defines a pedalboard — a signal chain of multiple `.pedal` files processed in series.

### Structure

```
board "<Name>" {
  <id>: "<path_to_pedal_file>"
  <id>: "<path_to_pedal_file>" { <Knob> = <value>, ... }
}
```

- Pedals are listed in signal-chain order (top = first in chain)
- Each entry: `<id>: "<path>"` with an optional knob overrides block
- Paths are resolved relative to the `.board` file's directory
- Comments use `#`

### Knob Overrides

Override default knob values inline with `{ Knob = value, ... }`:

```
ts: "tube_screamer.pedal" { Drive = 0.6, Level = 0.7 }
```

### Complete Example

```
# Blues Rig — classic three-pedal chain
board "Blues Rig" {
  ts: "tube_screamer.pedal" { Drive = 0.6, Level = 0.7 }
  bd: "blues_driver.pedal" { Gain = 0.5, Level = 0.6 }
  comp: "dyna_comp.pedal" { Sensitivity = 0.4, Output = 0.8 }
}
```

---

## CLI Usage

### Offline Processing

```bash
# Single pedal
pedalkernel process <file.pedal> <input.wav> <output.wav> [Knob=value ...]

# Pedalboard
pedalkernel process <file.board> <input.wav> <output.wav> [pedal_id.Knob=value ...]
```

**Pedal knob overrides** use `Knob=value`:
```bash
pedalkernel process tube_screamer.pedal input.wav out.wav Drive=0.9 Level=0.5
```

**Board knob overrides** use `pedal_id.Knob=value` (the `pedal_id` matches the identifier in the `.board` file):
```bash
pedalkernel process blues_rig.board input.wav out.wav ts.Drive=0.9 bd.Gain=0.7
```

### Interactive TUI (requires JACK)

```bash
# Single pedal
pedalkernel tui <file.pedal>

# Pedalboard
pedalkernel tui <file.board>
```

The TUI provides:
1. **Port selection** — choose JACK input/output ports
2. **Live control** — adjust knobs in real-time while audio plays

The pedalboard TUI adds a chain bar showing all pedals, with `[`/`]` to switch between pedals and `Space` to bypass individual pedals.
