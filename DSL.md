# PedalKernel DSL Reference

PedalKernel uses two file formats: `.pedal` files define individual guitar pedal circuits, and `.board` files chain multiple pedals into a pedalboard.

## `.pedal` File Format

A `.pedal` file describes a guitar pedal's circuit topology as a netlist. The compiler transforms this into a real-time Wave Digital Filter (WDF) audio processor.

### Structure

```
pedal "<Name>" {
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
| Photocoupler | `photocoupler(<model>)` | Vactrol / optocoupler (controlled resistance) |
| Neon bulb | `neon()` or `neon(<model>)` | Neon lamp for relaxation oscillators |
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
