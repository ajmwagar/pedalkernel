# PedalKernel Components Reference

This is the complete reference for all component types available in PedalKernel DSL. Use these in your `.pedal` files to build circuits from real electronic components.

## Passives

| Component | Syntax | What it does |
|-----------|--------|-------------|
| Resistor | `resistor(4.7k)` | Sets impedance, biasing, gain structure |
| Capacitor | `cap(220n)` | Frequency-dependent -- shapes the EQ curve |
| Inductor | `inductor(100m)` | Wah-style resonant peaks |
| Potentiometer | `pot(500k)` | Variable resistance, bound to knobs |
| Zener diode | `zener(4.7)` | Voltage clamp at a specific breakdown voltage |

## Diodes and clipping

| Component | Syntax | What it does |
|-----------|--------|-------------|
| Diode pair | `diode_pair(silicon\|germanium\|led)` | Symmetric clipping -- the core of overdrive |
| Single diode | `diode(silicon\|germanium\|led)` | Asymmetric clipping -- fuzz character |

## Transistors

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

## Op-amps

| Component | Syntax | Character |
|-----------|--------|-----------|
| Op-amp | `opamp(tl072)` | `tl072` (clean, fast), `jrc4558` (warm, compressed), `lm308` (slow slew -- the RAT sound), `lm741`, `ne5532` (low noise), `rc4558`, `tl082`, `op07` |
| OTA | `opamp(ca3080)` | CA3080 operational transconductance amplifier |

## Vacuum tubes

| Component | Syntax | Character |
|-----------|--------|-----------|
| Triode | `triode(12ax7)` | `12ax7` (high gain), `12at7` (medium), `12au7` (low), `12ay7` (Fender tweed), `12bh7` (high current), `6386` (variable-mu) |
| Pentode | `pentode(el34)` | `ef86` (Vox preamp), `el84` (Vox output), `6l6gc`/`5881`/`kt66` (Fender Twin, Dumble), `el34`/`6ca7`/`kt77` (Marshall), `6550`/`kt88`/`kt90` (Ampeg SVT), `6aq5a`, `6973` |

The pentode Koren models capture the real differences between tube types -- 6L6GC's gradual Fender compression vs EL34's sharp Marshall breakup vs 6550's extended clean headroom.

## Delay and modulation

| Component | Syntax | What it does |
|-----------|--------|-------------|
| BBD | `bbd(mn3207)` | Bucket-brigade delay: `mn3207` (1024-stage, Boss CE-2 chorus), `mn3007` (low-noise, Boss DM-2), `mn3005` (4096-stage, Memory Man) |
| Delay line | `delay_line(1ms, 500ms, allpass, tape_oxide)` | Composable WDF delay with configurable range, interpolation (`linear`, `allpass`, `cubic`), and medium simulation |
| Tap | `tap(DL1, 0.75)` | Tap point on a delay line at a fractional position -- build multi-tap delays, ping-pong, rhythmic patterns |
| LFO | `lfo(sine, 10k, 100n)` | Modulation from physical RC timing. Waveforms: `sine`, `triangle`, `square`, `saw_up`, `saw_down`, `sample_and_hold` |
| Envelope follower | `envelope_follower(10k, 100n, 100k, 1u, 47k)` | Dynamics-reactive modulation from RC attack/release networks |

The delay line system models physical medium degradation -- magnetic tape oxide causes frequency-dependent high-frequency loss (like a real Roland RE-201), BBD charge leakage causes amplitude decay, and digital quantization models PT2399-style bit reduction. Medium presets: `re201`, `echoplex`, `lo_fi`.

## Opto and neon

| Component | Syntax | What it does |
|-----------|--------|-------------|
| Vactrol | `photocoupler(vtl5c3)` | Optical compression: `vtl5c3`, `vtl5c1`, `nsl32`, `t4b` (LA-2A) |
| Neon bulb | `neon(ne2)` | Vintage tremolo oscillator: `ne2`, `ne51`, `ne83` |

## Transformers and switching

| Component | Syntax | What it does |
|-----------|--------|-------------|
| Transformer | `transformer(15:1, 10H, ct_primary)` | Audio/output transformer with configurable turns ratio, inductance, and winding types (standard, center-tap, push-pull) |
| Switched cap | `cap_switched([100n, 220n, 470n])` | Capacitor with switchable values |
| Switched inductor | `inductor_switched([100m, 200m])` | Inductor with switchable values |
| Switched resistor | `resistor_switched([1k, 2.2k, 4.7k])` | Resistor with switchable values |
| Rotary switch | `rotary_switch([C_bright, C_normal, C_dark])` | Controls which switched component value is active |
| Switch | `switch(2)` | Simple SPST/SPDT mechanical switch |

## Synth ICs

| Component | Syntax | What it does |
|-----------|--------|-------------|
| VCO | `vco(cem3340)` | Voltage-controlled oscillator: `cem3340`, `as3340`, `v3340` |
| VCF | `vcf(cem3320)` | Voltage-controlled filter: `cem3320`, `as3320` |
| VCA | `vca(ssm2164)` | Voltage-controlled amplifier: `ssm2164`, `v2164` |
| Comparator | `comparator(lm311)` | `lm311`, `lm393` |
| Analog switch | `analog_switch(cd4066)` | `cd4066`, `dg411` |
| Tempco resistor | `tempco(1k, 3300)` | Temperature-compensating resistor for exponential converters |

## Engineering notation

`100p` = 100 pF, `220n` = 220 nF, `2.2u` = 2.2 uF, `100m` = 100 mH, `4.7k` = 4.7 kOhm, `1M` = 1 MOhm

## Wiring

Signal flow uses `->`. Comma-separated pins at a junction are electrically connected:

```
in -> C1.a              # Signal enters the coupling cap
C1.b -> R1.a, D1.a      # Cap output goes to both the resistor and diode
D1.b -> gnd              # Diode clips to ground
```

Reserved nodes: `in`, `out`, `gnd`, `vcc`

## Controls

Map physical pot positions to named knobs:

```
Gain.position -> "Drive" [0.0, 1.0] = 0.5
```
