# PedalKernel DSL Reference

PedalKernel uses two file formats: `.pedal` files define individual guitar pedal circuits, and `.board` files chain multiple pedals into a pedalboard.

## `.pedal` File Format

A `.pedal` file describes a guitar pedal's circuit topology as a netlist. The compiler transforms this into a real-time Wave Digital Filter (WDF) audio processor.

The keywords `pedal`, `synth`, and `equipment` are interchangeable — all produce the same AST. Use `synth` for synthesizer modules and `equipment` for studio gear like compressors and preamps.

### Structure

```
pedal "<Name>" {
  supply <voltage>V        # single supply (optional — defaults to 9V)
  # OR for multiple rails:
  supplies {
    <rail_name>: <voltage>V [{ sag params }]
    ...
  }
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

**Supply block syntax** for explicit sag modeling:

```
pedal "Tube Amp Sag" {
  supply 480V {
    impedance: 150      # ohms (rectifier + transformer + ESR)
    filter_cap: 40u     # main filter capacitance in farads
    rectifier: tube     # or solid_state
  }
  ...
}
```

| Parameter | Description | Typical Values |
|-----------|-------------|----------------|
| `impedance` | Supply output impedance | Tube rectifier: 50–200Ω, solid-state: 1–10Ω |
| `filter_cap` | Main filter capacitance | 40µF (vintage) to 220µF (modern) |
| `rectifier` | Rectifier type: `tube` or `solid_state` | `tube` for sag, `solid_state` for stiff supply |

### Multiple Supply Rails

For circuits requiring multiple voltage rails (bipolar op-amp supplies, tube amps with separate B+ voltages, or isolated supply domains), use the `supplies { }` block:

```
pedal "Bipolar Op-Amp Circuit" {
  supplies {
    V+: 15V
    V-: -15V           # Negative voltages supported
  }
  components {
    R1: resistor(10k)
    U1: opamp(tl072)
  }
  nets {
    R1.a -> V+         # Connect to positive rail
    R1.b -> V-         # Connect to negative rail
    ...
  }
}
```

**Tube amplifier with multiple rails:**

```
pedal "Tube Preamp Multi-Rail" {
  supplies {
    B+: 300V { impedance: 100, rectifier: tube }
    bias: -50V         # Grid bias supply
    filament: 6.3V     # Heater supply
  }
  components {
    V1: triode(12ax7)
    R1: resistor(100k)
    ...
  }
  nets {
    V1.plate -> R1.a
    R1.b -> B+         # Plate load to B+ rail
    ...
  }
}
```

**Key features:**
- Rail names can include `+` or `-` suffixes (e.g., `V+`, `V-`, `B+`)
- Negative voltages are supported (e.g., `-15V`, `-50V`)
- Each rail can have its own sag parameters
- Rails are referenced in `nets` just like `vcc` and `gnd`

**Backwards compatibility:** The single `supply 9V` syntax still works and creates a "vcc" rail:

```
# These are equivalent:
supply 9V
# Same as:
supplies { vcc: 9V }
```

If no supply is specified, the compiler defaults to 9V on a "vcc" rail.

### Components

Each component is declared as `<id>: <type>(<params>)`.

| Type | Syntax | Description |
|------|--------|-------------|
| Resistor | `resistor(<value>)` | Fixed resistor |
| Capacitor | `cap(<value>)` | Capacitor |
| Inductor | `inductor(<value>)` | Inductor |
| Switched Resistor | `resistor_switched(<v1>, <v2>, ...)` | Resistor with switchable values |
| Switched Capacitor | `cap_switched(<v1>, <v2>, ...)` | Capacitor with switchable values |
| Switched Inductor | `inductor_switched(<v1>, <v2>, ...)` | Inductor with switchable values |
| Potentiometer | `pot(<value>)` | Variable resistor (controllable knob) |
| Diode pair | `diode_pair(<type>)` | Symmetric clipping diode pair |
| Single diode | `diode(<type>)` | Asymmetric single diode |
| Zener diode | `zener(<voltage>)` | Zener diode with breakdown voltage (e.g., `zener(5.1)`) |
| NPN transistor | `npn()` or `npn(<type>)` | NPN BJT (modeled as gain stage) |
| PNP transistor | `pnp()` or `pnp(<type>)` | PNP BJT (modeled as gain stage) |
| N-channel MOSFET | `nmos(<model>)` | N-channel MOSFET |
| P-channel MOSFET | `pmos(<model>)` | P-channel MOSFET |
| Op-amp | `opamp()` or `opamp(<type>)` | Operational amplifier |
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
| Switch | `switch(<positions>)` | Simple n-position mechanical switch |
| Rotary Switch | `rotary(<label1>, <label2>, ...)` | Labeled rotary switch |
| Analog Switch | `switch(<model>)` | Analog switch IC (CD4066, DG411) |
| VCO | `vco(<model>)` | Voltage-controlled oscillator IC |
| VCF | `vcf(<model>)` | Voltage-controlled filter IC |
| VCA | `vca(<model>)` | Voltage-controlled amplifier IC |
| Comparator | `comparator(<model>)` | Comparator IC |
| Matched NPN | `matched_npn(<model>)` | Matched NPN transistor pair/array |
| Matched PNP | `matched_pnp(<model>)` | Matched PNP transistor pair |
| Tempco Resistor | `tempco(<resistance>, <ppm>)` | Temperature-compensating resistor |
| Transformer | `transformer(<ratio>, <inductance> [, ...])` | Audio transformer (see below) |

**Engineering notation** is supported for component values:

| Suffix | Multiplier | Example |
|--------|-----------|---------|
| `p` | 10^-12 (pico) | `100p` = 100 pF |
| `n` | 10^-9 (nano) | `47n` = 47 nF |
| `u` | 10^-6 (micro) | `10u` = 10 uF |
| `m` | 10^-3 (milli) | `100m` = 100 mH |
| `k` | 10^3 (kilo) | `4.7k` = 4.7 kOhm |
| `M` | 10^6 (mega) | `1M` = 1 MOhm |
| `inf` | ∞ (infinity) | Open circuit |

The `inf` keyword represents infinite impedance (open circuit). This is useful for switched components where one position should disconnect the signal:

```
R_sel: resistor_switched([10k, inf, 47k])  # Position 1 = open circuit
```

**Capacitor types**: `film` (default), `electrolytic`, `ceramic`, `tantalum`

Capacitors support optional parasitic parameters:
```
C1: cap(22u)                              # Film cap (default), ideal
C2: cap(22u, electrolytic)                # Electrolytic, default parasitics
C3: cap(22u, electrolytic, leakage: 100k) # With explicit leakage resistance
C4: cap(22u, electrolytic, leakage: 10k, da: 0.05)  # With dielectric absorption
```

| Parameter | Description | Typical Values |
|-----------|-------------|----------------|
| `leakage` | Parallel leakage resistance (Ω) | 100kΩ (worn) to 10MΩ (new electrolytic) |
| `da` | Dielectric absorption coefficient (0-1) | 0.01-0.05 (electrolytic), <0.001 (film) |

**Diode types**: `silicon`, `germanium`, `led`

**Zener voltages**: Common values are 3.3, 4.7, 5.1, 5.6, 6.2, 9.1, 12 (in volts)

**JFET models**: `j201`, `2n5457`, `2n5460`, `2sk30` (or `2sk30a`, `2sk30-gr`, `2sk30-y`, `2sk30-bl` for graded variants)

**Triode types**: `12ax7`, `12at7`, `12au7`, `12bh7`, `12ay7`, `6386`

| Type   | µ (mu) | Use case                              |
|--------|--------|---------------------------------------|
| 12AX7  | 100    | High-gain preamp, voltage amplifier   |
| 12AT7  | 60     | Medium gain, phase inverter, driver   |
| 12AU7  | 17     | Low gain, cathode follower            |
| 12BH7  | 17     | High-current cathode follower         |
| 12AY7  | 44     | Lower-gain preamp (cleaner)           |
| 6386   | ~50    | Remote-cutoff (variable-mu) triode    |

European equivalents: `ecc83` (12AX7), `ecc81` (12AT7), `ecc82` (12AU7), `6072` (12AY7)

The **6386** is a remote-cutoff (variable-mu) dual triode used in the Fairchild 670 compressor. Unlike regular triodes, its mu varies with grid bias (~50 at low bias to ~5 at high bias), enabling gain control via bias modulation.

Triode pins: `.grid`, `.plate`, `.cathode`

**Pentode types**: `ef86`, `el84`, `el34`, `6l6gc`, `kt66`, `6aq5a`, `6973`, `6550`

| Type   | Max Watts | Use case                              |
|--------|-----------|---------------------------------------|
| EF86   | 1.2       | Low-noise preamp (Vox, Ampeg)         |
| EL84   | 12        | Small power amp (Champ, AC15)         |
| EL34   | 25        | Power amp (Marshall, Hiwatt)          |
| 6L6GC  | 30        | Power amp (Bassman, Twin)             |
| KT66   | 35        | Power amp (early Marshall, Quad)      |
| 6AQ5A  | 12        | Output stage, small amps              |
| 6973   | 12        | Power amp (Ampeg Reverberocket)       |
| 6550   | 35        | High-power output (SVT, Hiwatt)       |

Equivalents and variants:
- `6267` = EF86, `6bq5` = EL84, `6ca7` = EL34, `5881` = 6L6GC
- `kt77` = EL34 equivalent, `kt88`/`kt90` = 6550 equivalents

Pentode pins: `.grid`, `.plate`, `.cathode`, `.screen`

The screen grid requires a dropping resistor from B+ and bypass cap to ground:
```
vcc -> R_screen.a
R_screen.b -> V1.screen, C_screen.a
C_screen.b -> gnd
```

**Photocoupler models**: `vtl5c3`, `vtl5c1`, `nsl32`, `t4b`

**Neon bulb models**: `ne2` (default), `ne51`, `ne83`
- NE-2: 90V striking, 60V maintaining — standard miniature neon
- NE-51: 95V striking, 65V maintaining — higher current
- NE-83: 65V striking, 50V maintaining — lower voltage variant

Neon bulbs are used in vintage tremolo circuits (Fender Vibrato, Wurlitzer) as part of relaxation oscillators. When paired with an LDR (photocoupler), they create smooth optical modulation.

**LFO waveforms**: `sine`, `triangle`, `square`, `saw_up`, `saw_down`, `sample_hold`

**Switched components**: Values can be provided with or without brackets:
```
R_sel: resistor_switched(12k, 6.8k, 3.9k, 1.5k)
C_lf: cap_switched([27n, 68n, 220n, 1.5u])
L_hf: inductor_switched(27m, 47m, 82m, 150m)
```
Position is controlled via `.position` property in the controls section.

**MOSFET models**:
| Model | Type | Use case |
|-------|------|----------|
| 2n7000 | N-channel | Small-signal switching |
| irf520 | N-channel | Medium power switching |
| bs250 | P-channel | Small-signal switching |
| irf9520 | P-channel | Medium power switching |

**Op-amp models**: `tl072`, `tl082`, `jrc4558` (or `4558`), `rc4558`, `lm308`, `lm741`, `ne5532`, `ca3080`, `op07`

**VCO models**: `cem3340`, `as3340`, `v3340`
- CEM3340 — Curtis VCO (Prophet 5, Memorymoog)
- AS3340 / V3340 — Modern clones (Alfa RPAR, CoolAudio)

**VCF models**: `cem3320`, `as3320`
- CEM3320 — Curtis 4-pole lowpass (Prophet 5, Oberheim OB-Xa)
- AS3320 — Alfa RPAR clone

**VCA models**: `ssm2164`, `v2164`
- SSM2164 — Quad exponential VCA (Eurorack standard)
- V2164 — CoolAudio clone

**Comparator models**: `lm311`, `lm393`
- LM311 — Single comparator, open-collector output
- LM393 — Dual comparator

**Analog switch models**: `cd4066`, `dg411`
- CD4066 — Quad bilateral switch (~100Ω on-resistance)
- DG411 — Quad SPST analog switch (~25Ω on-resistance)

**Matched transistor models**: `ssm2210`, `ca3046`, `lm394`, `that340`
- SSM2210 — Matched dual NPN for V/Oct tracking
- CA3046 — 5-NPN transistor array (common substrate)
- LM394 — Supermatch pair, ultralow Vbe offset
- THAT340 — Modern matched quad NPN

**Switch types**:
```
SW1: switch(3)                            # 3-position mechanical switch
MODE: rotary("Clean", "Crunch", "Lead")   # Labeled rotary switch
AS1: switch(cd4066)                       # Analog switch IC
```

**Temperature-compensating resistor**:
```
RT1: tempco(2k, 3500)   # 2kΩ nominal, 3500 ppm/°C
```
Used in exponential converters for temperature compensation.

**Transformer syntax**:
```
# Basic: ratio, primary inductance
T1: transformer(10:1, 2H)

# With parasitics: ratio, inductance, DCR, parasitic capacitance
T2: transformer(1:4, 2H, 75, 200p)

# Center-tapped secondary
T3: transformer(1:1, 4H, ct)

# Push-pull primary, center-tapped secondary
T4: transformer(1:1, 4H, pp, ct)

# Named parameters
T5: transformer(10:1, 2H, dcr=75, Cp=200p, k=0.98)
```

Winding modifiers: `ct` (center-tap secondary), `pp` (push-pull primary), `ct_primary` (center-tap primary)

### Nets

Nets describe how component pins connect. Each net is `<from> -> <to1>, <to2>, ...`.

**Reserved nodes**:
- `in` — audio input
- `out` — audio output
- `gnd` — ground reference
- `vcc` — default power supply rail
- `fx_send` — effects loop send (for amp-style FX loops)
- `fx_return` — effects loop return (for amp-style FX loops)
- Named supply rails (e.g., `V+`, `V-`, `B+`) — when using `supplies { }` block

**Component pins** use dot notation: `C1.a`, `R1.b`, `D1.a`, `Q1.base`.

Two-terminal components (resistors, capacitors, inductors, diodes) have pins `a` and `b`. Potentiometers have pins `a` and `b` (2-terminal variable resistor), or `a`, `w` (wiper), and `b` (3-terminal pot for crossfade/blend). Transistors have `base`, `collector`, `emitter`. JFETs have `gate`, `drain`, `source`. Op-amps have `pos`, `neg`, `out`. LFOs and envelope followers have `out` (modulation output) which connects to modulation inputs like `<jfet>.vgs` or `<photocoupler>.led`.

**3-terminal pots:** When a pot uses the `.w` (wiper) pin in nets, it is modeled as two linked variable resistors sharing the wiper node. As position increases, R(a→w) increases and R(w→b) decreases, maintaining R(a→w) + R(w→b) = max_R. This enables crossfade/blend topologies where signals connect to both `.a` and `.b` with the output taken from `.w`.

```
in -> C1.a                       # input to capacitor pin a
C1.b -> R1.a, D1.a              # capacitor pin b fans out to R1 and D1
D1.b -> gnd                     # diode to ground
R1.b -> out                     # resistor to output
```

**Dynamic routing with `fork()`**: Routes signal to different destinations based on a switch position.

```
<from> -> fork(<switch_id>, [<dest1>, <dest2>, ...])
```

The switch component controls which destination receives the signal:
- Position 0 → routes to `dest1`
- Position 1 → routes to `dest2`
- etc.

```
# Route signal based on MODE switch (3-position)
stage1.out -> fork(MODE, [clean.a, crunch.a, lead.a])

# Mute switch — routes to output or ground
amp.out -> fork(MUTE, [out, gnd])

# Time constant selector for phaser stages
B7.a -> fork(SW_time, [B8.a, B9.a])
```

This enables topology switching for multi-mode circuits without runtime recompilation.

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

### Trims Section

Internal trim pots for factory adjustments (not user-facing):

```
trims {
  R_trim.position -> "Bias Adjust" [0.0, 1.0] = 0.5
  R_cal.position -> "Calibration" [0.0, 1.0] = 0.5
}
```

Same syntax as `controls`, but trims are not exposed in the UI.

### Monitors Section

For real-time metering visualization (VU meters, gain reduction, etc.):

```
monitors {
  V1.plate_current -> "Tube 1" [vu]
  output -> "Output Level" [ppm]
  GR.reduction -> "Gain Reduction" [gr]
  V2.plate_current -> "Tube 2" [glow]
  supply -> "B+ Sag" [sag]
}
```

**Meter types**:
| Type | Description |
|------|-------------|
| `vu` | VU meter (300ms rise/fall, RMS-responding) |
| `ppm` | Peak Programme Meter (10ms rise, 1.5s fall) |
| `peak` | True peak with hold |
| `gr` | Gain reduction meter (for compressors) |
| `glow` | Tube glow visualization |
| `sag` | Supply sag indicator |

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

### Included Examples

**Pedals** (`examples/pedals/`):

| File | Pedal | Type |
|------|-------|------|
| `overdrive/tube_screamer.pedal` | Tube Screamer TS-808 | Overdrive |
| `overdrive/blues_driver.pedal` | Boss Blues Driver | Overdrive |
| `overdrive/klon_centaur.pedal` | Klon Centaur | Overdrive |
| `overdrive/fulltone_ocd.pedal` | Fulltone OCD | Overdrive |
| `fuzz/fuzz_face.pedal` | Fuzz Face | Fuzz |
| `fuzz/big_muff.pedal` | Big Muff | Fuzz |
| `distortion/proco_rat.pedal` | ProCo RAT | Distortion |
| `compressor/dyna_comp.pedal` | MXR Dyna Comp | Compressor |
| `modulation/boss_ce2.pedal` | Boss CE-2 | Chorus |
| `phaser/phase90.pedal` | MXR Phase 90 | Phaser |

**Amplifiers** (`examples/amps/`):

| File | Amp |
|------|-----|
| `tweed_deluxe_5e3.pedal` | Fender Tweed Deluxe 5E3 |
| `bassman_5f6a.pedal` | Fender Bassman 5F6-A |
| `marshall_jtm45.pedal` | Marshall JTM45 |

**Synthesizer Modules** (`examples/synths/`):

| File | Module |
|------|--------|
| `cem3340_vco.pedal` | CEM3340 VCO |
| `moog_ladder_vcf.pedal` | Moog Ladder VCF |
| `minisynth.pedal` | Complete mini-synth

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

### Validation

```bash
# Validate a single file
pedalkernel validate my_pedal.pedal

# Validate a directory (recursive)
pedalkernel validate examples/

# Validate with glob pattern
pedalkernel validate "examples/**/*.pedal"

# Auto-fix obvious issues (pin renames, etc.)
pedalkernel validate my_pedal.pedal --fix
```

### Interactive TUI (requires JACK)

```bash
# Single pedal
pedalkernel tui <file.pedal>

# Pedalboard
pedalkernel tui <file.board>

# Directory browser (opens folder picker)
pedalkernel tui examples/

# Loop a WAV file as input instead of JACK port
pedalkernel tui <file.pedal> --input test.wav
```

The TUI provides:
1. **Port selection** — choose JACK input/output ports
2. **Live control** — adjust knobs in real-time while audio plays

The pedalboard TUI adds a chain bar showing all pedals, with `[`/`]` to switch between pedals and `Space` to bypass individual pedals.
