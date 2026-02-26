# PedalKernel Synth Modeling — Implementation Plan

**Goal**: Extend pedalkernel so that synthesizer circuits (VCO, VCF, VCA, ADSR, etc.) can be described in the `.pedal` DSL, compiled to real-time audio via WDF, exported to KiCad for PCB layout, and ordered as a Mouser BOM. Same file, same tone — from DSL to PCB.

---

## Design Principles

1. **Every component is a real part.** No abstract DSP blocks. A VCO is a CEM3340 IC (or AS3340/V3340 clone) with external R/C values that set its behavior. An ADSR is resistors + diodes + a cap + a comparator. A Moog filter is 8 matched transistors + 4 caps.

2. **DSL describes circuits, not algorithms.** The `.pedal` syntax already handles this — component declarations with real values, net connections with real wiring. Synths use the same paradigm.

3. **Full pipeline for every component.** Each new `ComponentKind` variant gets:
   - DSL parser (nom)
   - WDF/audio model (elements)
   - KiCad symbol mapping (kicad.rs)
   - BOM entry with Mouser P/N (hw.rs)
   - Compiler support (compiler.rs)

4. **`synth` keyword alongside `pedal`.** The DSL gains a `synth "Name" { ... }` top-level form. Internally it produces the same `PedalDef` AST — the compiler doesn't care whether it's a pedal or a synth. The keyword is for documentation and UI (different TUI layout, different KiCad sheet template).

---

## Phase 1: New DSL Reserved Nodes & Keyword

### 1a. `synth` keyword
Add `synth` as an alias for `pedal` in the top-level parser. Both produce `PedalDef`.

```
synth "Minimod" {
  components { ... }
  nets { ... }
  controls { ... }
}
```

**Files**: `dsl.rs` (parser), add `parse_synth_header` alongside `parse_pedal_header`

### 1b. New reserved nodes for CV/Gate

Current reserved nodes: `in`, `out`, `gnd`, `vcc`, `fx_send`, `fx_return`

Add:
- `gate` — note on/off signal (0V or +5V). Maps to a physical gate jack on the PCB.
- `cv_pitch` — pitch control voltage (V/Oct standard: 0V = base note, +1V = octave up). Maps to a CV input jack.
- `cv_mod` — modulation CV input (mod wheel, aftertouch, external CV). Maps to a jack.
- `cv_filter` — filter cutoff CV. Maps to a jack.

In the WDF engine, these are voltage sources driven by MIDI-derived values (or by the TUI for testing). On the KiCad side, they're 3.5mm mono jacks (Eurorack) or 1/4" jacks.

**Files**: `dsl.rs` (RESERVED_NODES), `compiler.rs` (source injection), `kicad.rs` (jack symbols)

---

## Phase 2: New Component Types — Synth ICs

Each is a real IC with real behavior, a KiCad symbol, and a Mouser part number.

### 2a. CEM3340 / AS3340 / V3340 — Voltage-Controlled Oscillator

**DSL syntax**: `vco(cem3340)`, `vco(as3340)`, `vco(v3340)`

**Pins**: `.cv` (exponential V/Oct input, pin 15), `.saw` (sawtooth output), `.pulse` (pulse/square output), `.tri` (triangle output), `.sync` (hard sync input), `.pw` (pulse width CV)

**WDF model**: The CEM3340 internally contains an exponential converter + integrator + comparator. The model:
- Takes CV input, applies exponential V/Oct conversion (frequency = base_freq × 2^(cv/1.0))
- Generates sawtooth core (phase accumulator parameterized by timing capacitor value from external C)
- Derives triangle (from sawtooth via internal waveshaper) and pulse (comparator vs PW threshold)
- External components (1nF timing cap, 100k CV summing resistors, 1k output resistors) parameterize behavior

**KiCad**: `Synth:CEM3340` (or `Analog:CEM3340` — will use custom KiCad library or reference existing community symbol)

**BOM**: AS3340 (Alfa RPAR) — active production, available at multiple suppliers. CoolAudio V3340 also available.

**Key external components** (these go in the `.pedal` file as separate component declarations):
- C_timing: `cap(1n)` — polystyrene timing capacitor (sets frequency range)
- R_cv: `resistor(100k)` — CV input summing resistor (1V/Oct scaling)
- R_out_saw: `resistor(1k)` — sawtooth output protection
- R_tune: `pot(100k)` — coarse tune
- R_fine: `pot(10k)` — fine tune

### 2b. CEM3320 / AS3320 — Voltage-Controlled Filter (4-pole)

**DSL syntax**: `vcf(cem3320)`, `vcf(as3320)`

**Pins**: `.in` (signal input), `.out` (filtered output), `.cv` (cutoff frequency CV), `.res` (resonance/Q CV)

**WDF model**: 4-pole lowpass with voltage-controlled cutoff. The IC internally uses OTA-based integrators. Model behavior:
- Cutoff frequency controlled exponentially by CV (V/Oct or Hz/V depending on external resistor)
- Resonance from 0 (no peak) to self-oscillation
- 24dB/oct rolloff characteristic
- External R/C values set frequency range and input scaling

**KiCad**: `Synth:CEM3320` or `Analog:AS3320`

**BOM**: AS3320 (Alfa RPAR)

### 2c. SSM2164 / V2164 — Quad VCA

**DSL syntax**: `vca(ssm2164)`, `vca(v2164)`

**Pins**: Per section (4 VCAs in one IC): `.in1`/`.out1`/`.cv1` through `.in4`/`.out4`/`.cv4`

**WDF model**: Each section is a current-controlled amplifier with exponential CV response. Model:
- Gain = exp(-cv / 30mV) — exponential current-mode control
- Low distortion at moderate levels, soft limiting at extremes
- External feedback resistors set gain range

**KiCad**: `Analog:SSM2164` or `Analog:V2164`

**BOM**: CoolAudio V2164 (available, pin-compatible with SSM2164)

### 2d. Comparator ICs

**DSL syntax**: `comparator(lm311)`, `comparator(lm393)`

**Pins**: `.pos` (non-inverting input), `.neg` (inverting input), `.out` (open-collector output)

**WDF model**: Binary output — high when pos > neg, low otherwise. Used in VCO reset circuits, Schmitt triggers, window comparators. Not a WDF root — just a signal routing element (like op-amps in non-OTA mode).

**KiCad**: `Comparator:LM311`, `Comparator:LM393`

**BOM**: LM311 (ON Semi, 595-LM311P), LM393 (TI, 595-LM393P)

### 2e. Analog Switch ICs

**DSL syntax**: `switch(cd4066)`, `switch(dg411)`

**Pins**: `.in1`/`.out1`/`.ctrl1` through `.in4`/`.out4`/`.ctrl4` (CD4066 is quad bilateral switch)

**WDF model**: When ctrl is high, channel is a low-resistance path (~100Ω for CD4066). When ctrl is low, channel is open (very high R). Used in sample-and-hold, gate-controlled routing, ADSR switching.

**KiCad**: `Analog_Switch:CD4066`, `Analog_Switch:DG411`

**BOM**: CD4066BE (TI, 595-CD4066BE)

### 2f. Matched Transistor Pairs/Arrays

**DSL syntax**: `matched_npn(ssm2210)`, `matched_npn(lm394)`, `npn_array(ca3046)`

**Pins**: Same as NPN but guaranteed matched Vbe. For CA3046: 5 transistors with `.q1_base`, `.q1_collector`, `.q1_emitter`, etc.

**WDF model**: Same as NPN but with tighter parameter matching (lower Vbe offset). Used in exponential converters where matching is critical for V/Oct tracking.

**KiCad**: `Transistor_BJT:SSM2210`, `Transistor_Array:CA3046`

**BOM**: SSM2210 (Analog Devices), CA3046 (Intersil/Renesas)

### 2g. Tempco Resistor

**DSL syntax**: `tempco(2k, 3500)` — 2kΩ nominal, +3500ppm/°C

**Pins**: `.a`, `.b` (standard 2-terminal)

**WDF model**: Resistance varies with temperature: R(T) = R_nom × (1 + α × (T - 25°C)). Used in exponential converters to compensate transistor Vbe temperature drift.

**KiCad**: `Device:R` with special value annotation

**BOM**: Tempco resistor 2kΩ +3500ppm/°C (multiple suppliers)

---

## Phase 3: Discrete Synth Circuits (Examples)

These use existing + new components wired together. No new component types needed — just example `.pedal` (or `.synth`) files demonstrating that the DSL can describe these circuits.

### 3a. Moog Transistor Ladder Filter

Built entirely from existing components:
- 8× `npn()` (matched pairs, or `npn_array(ca3046)`)
- 4× `cap(1n)` to `cap(10n)` (set cutoff range)
- Biasing resistor chain
- Feedback path with `pot()` for resonance

```
synth "Moog Ladder VCF" {
  components {
    # 4 differential pairs (ladder core)
    Q1: npn()   Q2: npn()
    Q3: npn()   Q4: npn()
    Q5: npn()   Q6: npn()
    Q7: npn()   Q8: npn()

    # Filter capacitors (set cutoff range)
    C1: cap(1n)   C2: cap(1n)   C3: cap(1n)   C4: cap(1n)

    # Biasing
    R_bias1: resistor(18k)
    R_bias2: resistor(18k)
    R_bias3: resistor(18k)
    R_bias4: resistor(18k)

    # Resonance feedback
    Resonance: pot(100k)

    # Input/output buffering
    U1: opamp(tl072)
  }
  nets {
    in -> Q1.base
    Q1.emitter -> C1.a, Q3.base
    ...
    Q7.collector -> C4.a
    C4.b -> U1.pos
    U1.out -> Resonance.a
    Resonance.b -> Q2.base    # Feedback for resonance
    cv_filter -> ...           # Cutoff CV controls bias current
  }
  controls {
    Resonance.position -> "Resonance" [0.0, 1.0] = 0.0
  }
}
```

### 3b. Discrete ADSR Envelope Generator

Built from existing + new components:
- Capacitor (envelope storage)
- Resistors (A/D/R time constants)
- Diodes (steering: attack charges through D1+R_atk, decay/release discharges through D2+R_dec)
- Comparator (sustain level detection, end-of-attack trigger)
- Analog switch (gate-controlled charge/discharge path)

### 3c. Complete Monosynth

Combines VCO (CEM3340) + VCF (discrete Moog ladder or CEM3320) + VCA (CA3080 OTA or SSM2164) + ADSR.

---

## Phase 4: Compiler Changes

### 4a. CV/Gate source injection
The compiler needs to recognize `gate`, `cv_pitch`, `cv_mod`, `cv_filter` as voltage sources driven by external input (MIDI or TUI). These behave like `VoltageSource` nodes in the WDF tree but are updated per-sample from the control system.

### 4b. VCO as a signal source
The CEM3340 model generates audio internally (it's an oscillator, not a filter). In the WDF tree, it acts like a `VoltageSource` whose voltage is the oscillator output. The compiler detects `vco()` components and creates an oscillator that feeds into the rest of the circuit.

### 4c. Multi-output components
The CEM3340 has 3 outputs (saw, pulse, tri). The compiler needs to handle components with multiple output pins, each carrying a different signal. This is similar to how op-amps have 3 pins.

### 4d. Gate-triggered behavior
ADSR envelopes and VCA gain are triggered by gate signals. The compiler needs to route the `gate` reserved node to components that respond to it (comparators, analog switches, ADSR ICs).

---

## Phase 5: KiCad & BOM Updates

### 5a. KiCad symbol mappings
Add to `kicad.rs`:
- `Vco(Cem3340)` → custom symbol or `Oscillator:CEM3340`
- `Vcf(Cem3320)` → `Filter:CEM3320` or `Analog:AS3320`
- `Vca(Ssm2164)` → `Analog:SSM2164`
- `Comparator(Lm311)` → `Comparator:LM311`
- `Switch(Cd4066)` → `Analog_Switch:CD4066`
- `MatchedNpn(Ssm2210)` → `Transistor_BJT:SSM2210`
- `Tempco(r, ppm)` → `Device:R` with special annotation

### 5b. BOM Mouser part numbers
Add to `hw.rs`:
- AS3340: Alfa RPAR (or specify manual sourcing like BBD chips)
- AS3320: Alfa RPAR
- V2164: CoolAudio
- LM311P: `595-LM311P`
- LM393P: `595-LM393P`
- CD4066BE: `595-CD4066BE`
- SSM2210: `584-SSM2210PZ`
- CA3046: manual sourcing (discontinued, NOS market)
- Tempco 2kΩ: specialist supplier

### 5c. CV/Gate jacks in KiCad
Reserved nodes `gate`, `cv_pitch`, `cv_mod`, `cv_filter` export as connector symbols (3.5mm Eurorack jacks or 1/4" jacks depending on config).

---

## Phase 6: Runtime & TUI

### 6a. MIDI input
Add optional MIDI support (via `midir` crate or JACK MIDI):
- Note on/off → gate voltage (0V/5V) + cv_pitch voltage (V/Oct)
- Mod wheel → cv_mod voltage (0-5V)
- Velocity → optional cv_velocity

### 6b. TUI for synths
The TUI already adapts to whatever controls exist. For synths, it would show:
- Knobs for cutoff, resonance, envelope times, etc.
- A virtual keyboard (or note trigger buttons)
- Gate indicator
- Waveform selector

### 6c. Polyphony
Multiple instances of the compiled synth running in parallel, each with independent gate/CV. Voice allocation logic in the runtime.

---

## Implementation Order

1. **Phase 1** (DSL): `synth` keyword, new reserved nodes — small, foundational
2. **Phase 2a** (VCO): CEM3340 component type + WDF model + KiCad + BOM — the oscillator is the core of any synth
3. **Phase 2d** (Comparator): LM311/LM393 — needed for discrete circuits
4. **Phase 2e** (Analog Switch): CD4066 — needed for ADSR and S&H circuits
5. **Phase 3a** (Example): Moog ladder filter — validates that existing components work for synth circuits
6. **Phase 2b** (VCF IC): CEM3320/AS3320 — IC-based filter alternative
7. **Phase 2c** (VCA IC): SSM2164/V2164 — dedicated VCA
8. **Phase 3b** (Example): Discrete ADSR — validates comparator + switch components
9. **Phase 3c** (Example): Complete monosynth — the full integration test
10. **Phase 4** (Compiler): CV/Gate injection, VCO as source, multi-output
11. **Phase 5** (KiCad/BOM): All new symbols and part numbers
12. **Phase 6** (Runtime): MIDI, TUI, polyphony

---

## What Stays the Same

- WDF engine (tree.rs, elements/) — no changes to the core DSP
- PedalProcessor trait — synths implement the same trait
- JACK real-time engine — same audio backend
- Oversampling, thermal model, tolerance engine — all applicable to synth circuits
- Pedalboard chaining — a synth into a pedal chain is a valid rig
