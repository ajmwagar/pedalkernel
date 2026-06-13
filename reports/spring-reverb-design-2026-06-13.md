# Spring Reverb Primitive — Design Doc (2026-06-13)

Status: PROPOSAL for review. Adds a `spring(tank_model)` behavioral-island component, a `SpringTank`
runtime element, and a `spring_lowering` compiler pass mirroring `bbd_lowering`/`vca_lowering`.

## 1. Research summary

### 1.1 Parker/Välimäki parametric model (the algorithm we adopt)
Välimäki, Parker & Abel, *Parametric Spring Reverberation Effect*, JAES 58(7/8), 2010
([AES e-lib 15511](https://www.aes.org/e-lib/online/browse.cfm?elib=15511)): a spring's impulse response is a
train of repeating **chirps** (pulse transits the spring ~30–56 ms, dispersion makes low frequencies arrive
later). The model is a **spectral delay filter** — a long cascade of identical *stretched* first-order
allpasses — inside a feedback loop with a delay line, damping filter, and loop gain. A second, cheaper
cascade models the weak high-frequency chirp above the transition frequency. Parker's DAFx follow-up
(*Efficient Dispersion Generation Structures for Spring Reverb Emulation*,
[CyberLeninka](https://cyberleninka.org/article/n/1369269)) shows group delay scales strongly with the
stretch factor and multirate tricks cut cost to ~1/3 — relevant later, not required for v1.
Published parameters (from the reference Matlab implementation,
[tomas-gajarsky/parametric-spring-reverb](https://github.com/tomas-gajarsky/parametric-spring-reverb)):
- Low-chirp cascade: **M = 100** stretched allpasses, coefficient **a1 = 0.75**.
- High-chirp cascade: **Mh = 200** allpasses, coefficient **0.6**, output mixed ~60 dB down (`g_high = g_low/1000`).
- Stretch factor **K = fs/(2·fC)** with transition frequency **fC = 4300 Hz**; integer part K1 = round(K)−1,
  fractional part as a Thiran fractional delay.
- Loop delay **Td = 56 ms**; loop gains **g_lf = −0.8**, **g_hf = −0.77** (negative — alternating echo polarity).
- DC blocker at 40 Hz; pre-EQ resonance ~95 Hz (B ≈ 130 Hz); slow random modulation of the loop delay
  (filtered noise, coeff 0.93) to blur late echoes into a tail.

### 1.2 Accutronics tank data (parameterization source)
From [Amplified Parts tank guides](https://www.amplifiedparts.com/tech-articles/accutronics-products-and-specifications)
and [sound-au.com](https://sound-au.com/articles/reverb.htm) (extracts; pages 403-block direct fetch):
- **Type 4**: 17" tank, 4 springs as 2 counter-wound coupled pairs (Fender standard). **Type 8**: 9.25"
  short tank, 3 springs. **Type 9**: 17", 6 springs in 3 coupled pairs in parallel (top of line).
- Decay (RT60): short 1.2–2.0 s, medium 1.75–3.0 s, long 2.75–4.0 s (part-number digit 4).
- Transit times ~30–40 ms per spring (e.g. 33 ms typical); springs within a tank are slightly detuned to
  decorrelate. Dispersion transition fC ≈ 2–4 kHz (4.3 kHz measured in the JAES paper's tank).
- Input coil Z: 8–1475 Ω (current-driven); output coil Z: 500 Ω–10 kΩ (voltage recovery into high-Z preamp).

### 1.3 Drive/recovery circuits (stay OUTSIDE the island)
- **RE-201** (service manual & [freestompboxes analysis](https://www.freestompboxes.org/viewtopic.php?t=29048)):
  reverb driver is a **TA7200P power-amp IC** driving the tank's low-Z input coil hard (current drive →
  coil saturation = "drip" when slammed); recovery is a small-signal transistor preamp (2SC828-R/2SA733-class,
  high gain, noise-limited). The drive nonlinearity lives in the driver stage, not the spring.
- **AKG BX20** ([GroupDIY](https://groupdiy.com/threads/technology-of-the-akg-bx-spring-reverbs-mainly-the-bx-20.37106/),
  [UA manual](https://help.uaudio.com/hc/en-us/articles/28320595155988-AKG-BX20-Spring-Reverb-Manual)): two
  long torsion-spring columns, **moving-coil** transducers at *both ends* of each spring, each with a drive
  coil + feedback coil; **motional feedback** electronically varies decay (~2–4.5 s) — i.e. decay control is
  a *circuit* property feeding back around the tank, further evidence dwell/decay belongs in circuit stages.

## 2. Engine architecture recap (what we mirror)
- Behavioral islands are `GraphRole::Virtual` components with a single `Behavioral` in→out edge —
  `Bbd` at `pedalkernel/src/compiler/components/delay.rs:18-115` (edge at :59-66, `modulation_sink` at :81-90).
- Lowering precedent: `pedalkernel/src/compiler/bbd_lowering.rs` — declaration-order instances
  (`bbd_component_ids` :47-54, `build_bbd_delay_lines` :57-71), boundary pins as stage terminals
  (`bbd_boundary_nodes` :79-91), galvanic-cluster **gap splitting** (`split_groups_at_behavioral_gaps`
  :108-179, shared by VCA), control/LFO binding (`bind_bbd_runtime` :189-305).
- Mandatory-lowering contract: `pedalkernel/src/compiler/vca_lowering.rs:49-76`
  (`behavioral_island_error`, `reject_unlowered_behavioral`) and wiring checks in `lower_vcas` :115-146.
- Pipeline integration points: `pedalkernel/src/compiler/spqr_build.rs:118-125` (build + reject),
  :293-299 (gap split), :331-339 (boundary terminals).
- Runtime serial-chain hook: `pedalkernel-rt/src/processor.rs:3784-3789` (BBD wet/dry after stage chain;
  `bbds` field :1237, `bbd_wet_mix` :1320). Delay infrastructure: ring buffer + Thiran allpass interpolation
  at `pedalkernel-rt/src/elements/nonlinear/delay.rs:277-305`; named-model precedent `BbdModel::mn3207()`
  etc. at `pedalkernel-rt/src/elements/nonlinear/bbd.rs:63-113`.

## 3. DSL design

```pedal
RV1: spring(type4)          # tank models: type4 | type8 | type9 | re201_tank
Q1.collector -> RV1.in      # driver transistor stage = drip/drive nonlinearity (circuit, not island)
RV1.out -> C12 -> Q2.base   # recovery preamp = ordinary circuit stage
RV1.dwell -> "Dwell"        # control pins (see §6)
```
Parser: add `parse_spring` + `SpringTankType` next to `parse_bbd`/`bbd_type` at
`pedalkernel/src/dsl.rs:1337-1351`, registered in the component alt-list at :2014. Pins: `in`, `out`
(aliases per `PinConfig`), control pins `dwell`, `damping`, `mix`. The component contributes one
`Behavioral` edge and `StampResult::Skip`, exactly like `Bbd`.

**Drive/recovery are deliberately NOT in the island.** The `.pedal` file wires real driver/recovery stages
(op-amp or BJT) around `spring()`; coil-overdrive "drip" emerges from the driver stage's existing nonlinear
WDF solve (BJT/op-amp clipping into the low-Z load), matching RE-201/BX20 topology. The island is the
*electromechanical spring pair only* — linear but dispersive.

## 4. Runtime element: `SpringTank` (pedalkernel-rt)

New file `pedalkernel-rt/src/elements/nonlinear/spring.rs` (registered in `elements/mod.rs`):

```text
SpringTankModel { num_springs, transit_ms[..], fc_hz, rt60_s, num_allpasses, ap_coeff,
                  hf_chirp_gain, damping_fc_hz, detune_cents }
SpringTank      { springs: [SpringLine; MAX_SPRINGS],  // fixed-capacity, no Vec growth post-new
                  dc_blocker, eq_peak, mod_noise_state, sample_rate }
SpringLine      { ap_z1: [Wave; M_MAX],   // stretched-allpass unit-delay states, M ≤ 128
                  k1_buf: RingBuf,        // shared K1-sample stretch delay (one per cascade) — see OQ2
                  loop_buf: RingBuf,      // Td·fs samples, Thiran fractional read (reuse delay.rs pattern)
                  damping_lpf, loop_gain, thiran_state }
```
Per-sample (constant cost): input → EQ peak (~95 Hz) → for each spring: loop-sum → M stretched allpasses
(`y = -a·x + z; z = x + a·y` per section, stretch via K1 delay) → Td loop delay (read with Thiran allpass
interp, slow noise-modulated ±few samples) → damping one-pole LPF → ×(−g_loop) feedback; outputs of springs
summed (per-spring transit detune decorrelates) → optional Mh/2.3 high-chirp path at −60 dB → DC blocker.
`g_loop` derived from RT60: `g = 10^(−3·Td/RT60)`. Named models:

| model        | springs | transit (ms)     | fC (Hz) | RT60 default | M (APs) | basis |
|--------------|---------|------------------|---------|--------------|---------|-------|
| `type4`      | 2 pairs | 33.0 / 37.5      | 4200    | 2.85 s       | 100     | JAES tank, long-decay 4AB3C1B |
| `type8`      | 3       | 30 / 32 / 34     | 3500    | 1.8 s        | 80      | short 9.25" tank, shorter springs |
| `type9`      | 3 pairs | 33 / 36.5 / 40   | 4000    | 2.5 s        | 100     | 6-spring parallel, densest |
| `re201_tank` | 2       | 31 / 35          | 3000    | 1.6 s        | 80      | small dark tank + bandwidth-limited recovery |

(Numbers grounded where sources allowed; calibration against measured IRs is OQ1.)

## 5. Lowering pass: `compiler/spring_lowering.rs`
Mirrors `bbd_lowering` exactly:
1. `spring_component_ids` / `build_spring_tanks` — declaration-order `Vec<SpringTank>`, index contract for
   `ControlTarget::SpringDwell(idx)/SpringDamping(idx)` and `ModulationTarget::SpringDwell`.
2. `spring_boundary_nodes` — `in`/`out` pins added to SPQR terminals (extend `spqr_build.rs:331-339`).
3. Reuse `bbd_lowering::split_groups_at_behavioral_gaps` (`spqr_build.rs:297-299` condition gains
   `|| !springs.is_empty()`) — driver stage and recovery stage become separate serial stages; the runtime
   bridges them: insert `for spring in &mut self.springs { wet = spring.process(signal); signal = dry·(1−mix) + wet·mix }`
   alongside the BBD hook at `processor.rs:3784-3789`.
4. `bind_spring_runtime` — pots wired to `RV.dwell`/`RV.damping` retarget via the `upsert_control` pattern
   (`bbd_lowering.rs:311-342`); mix pot via `pot_reaches_*_output` BFS (:477-547).
5. **Mandatory lowering**: extend `reject_unlowered_behavioral` (`vca_lowering.rs:64-76`) so a `spring()`
   with unwired `in`/`out` fails compile via `behavioral_island_error` — never ship a silent tank.

## 6. Control surface
- **Dwell/Drive** — physically the driver-stage gain (circuit pot). Island also exposes `dwell` as input
  trim for non-circuit patches; default 1.0.
- **Damping** — scales `damping_fc_hz` (loop LPF), i.e. how fast the tail darkens; 0..1 → 2·fc..fc/4 (log).
- **Mix** — wet/dry at the island bridge, like `bbd_wet_mix`.
- RT60 is a model constant per tank (matching real tanks); BX20-style variable decay later via motional
  feedback in the circuit (OQ4).

## 7. REALTIME contract (realtime-or-bust)
- **Constant per-sample cost**: fixed M allpasses/spring (worst case 6 springs × 100 APs ≈ 600 MAC+state ops
  ≈ 3–4 kFLOP/sample — between CE-2 (207×) and Tube Screamer (80×) per docs/performance.md:18-28).
- **Allocation-free process path**: all buffers sized in `new()` (max Td at max fs); `process()` does no
  alloc, no locks, no syscalls — same standard as `BbdDelayLine::process` (`bbd.rs:369-501`).
- **State memory bound**: ≤ 6 springs × (128 AP states + K1 buf ≤ 8 samples·128 + 56 ms loop buf) ≈ tens of
  kB at 48 kHz; noise modulation uses the xorshift PRNG pattern (`bbd.rs:559-567`), no `rand` crate.
- NaN/Inf guard on the loop feedback (flush state, as `read_allpass` does at `delay.rs:299-302`).
- Bench: add to `pedalkernel/examples/rt_bench.rs`; target **≥ 30× realtime single-core** at 48 kHz for a
  full driver + `spring(type4)` + recovery pedal.

## 8. Test plan (`tests/spring_tank.rs` + `tests/audio_analysis.rs` helpers)
1. **Chirp signature**: impulse → spectrogram-free check via `goertzel_mag` (audio_analysis.rs:212) at
   bands ×{0.5,1,2,4} kHz on windows around first arrival: group delay must *decrease* with frequency below
   fC (measured dispersion within ±15% of K·M-predicted curve); echo spacing ≈ 2·transit.
2. **RT60**: `energy` decay slope (audio_analysis.rs:171) over 0.5–2.5 s windows vs published decay class
   (type4 long: 2.75–4 s) within ±25%.
3. **Drive nonlinearity**: full pedal (driver BJT + spring + recovery) via `static_gain_curve`
   (audio_analysis.rs:675) — wet THD rises with drive level while island-alone THD stays ≈ 0 (linearity proof).
4. **Mandatory-lowering**: unwired `spring()` fails compile with the §4 error (mirror `tests/vca_lowering.rs`).
5. **Determinism + finiteness**: 10 s noise through 6-spring type9, all outputs finite, two runs bit-identical
   (note dyna_comp nondeterminism caveat in CLAUDE.md — assert here to catch regressions early).
6. **Bench**: rt_bench entry asserts ≥ 30× single-core.

## 9. What the RE-201 example gains
There is no RE-201 example yet — `delay.rs:621-630` (multi-head `Tap`) and `MediumPreset::Re201`
(`delay.rs:101-113`) cover the tape half only; the Reverb knob path has had no primitive. With `spring()`,
a future `examples/rack/re201_space_echo.pedal` becomes complete: `delay_line` + 3 taps for the heads,
`spring(re201_tank)` fed from the echo bus through a TA7200P-style driver stage, recovery preamp into the
mix — reproducing modes 5–11 (echo+reverb) and the dark, fast "boing" that defines the unit, with drip
intensity emerging from driver drive level rather than a baked-in parameter.

## 10. Open questions (for review)
- **OQ1**: Per-model transit times/fC are partly inferred — primary Accutronics datasheet tables were
  403-blocked. Calibrate against measured tank IRs (Parker's automated-calibration paper) before freezing constants?
- **OQ2**: Implement the stretch as one shared K1-sample delay per cascade (JAES structure, cheap) vs per-AP
  interleaved states (simpler code, M× memory)? Proposal: shared delay, matching the paper.
- **OQ3**: Is the −60 dB high-chirp cascade (Mh=200) worth ~2× the AP cost in v1, or ship low-chirp only
  behind a model flag? Proposal: flag, default off for type8/re201_tank.
- **OQ4**: BX20-style variable decay (motional feedback) — model as a `feedback` control pin on the island,
  or as a real feedback net around it (needs cross-stage feedback routing)? Deferred.
- **OQ5**: Should `spring()` reuse `CompiledPedal.bbd_wet_mix`-style global mix or per-instance mix? Proposal:
  per-instance field to avoid the BBD's shared-mix limitation.
