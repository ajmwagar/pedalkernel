# Outboard Gear Readiness Audit — 2026-06-12

Goal: assess what stands between the current WDF engine and reliably modeling
outboard production gear — EQs, compressors, limiters — with an eye toward
500-series modules and eventually 19" rack units. Branch:
`claude/kind-brahmagupta-7tjsyw` (audit baseline: main @ `01ac781`).

Method: code-level inventory of the DSL and engine, review of the 2026-06-09
SPICE validation reports, web research of the target circuits (Urei 1176,
Teletronix LA-2A, dbx 160, SSL G-bus, Pultec EQP-1A, API 550A, plus the
500-series VPR spec), then four new research-grounded example circuits built
and **measured** against the engine. Findings below are empirical unless
marked otherwise.

## Executive summary

The DSL is ahead of the engine. Every dynamics primitive a studio compressor
needs already *parses* — `envelope_follower`, `photocoupler(t4b)`,
`vca(ssm2164)`, OTA, vari-mu, JFET-as-resistor, sidechain blocks, multi-rate
subcircuits — but when wired into the four canonical outboard topologies,
**none of the three classic gain-reduction paths produces correct compression
today**:

| Gain element | DSL syntax | Measured behavior |
|---|---|---|
| FET shunt (1176) | `EF1.out -> J1.vgs` | **Inert** — 0.00 dB gain change between loud and quiet program |
| Opto cell (LA-2A) | `EF1.out -> PC1.led` | **Inverted** — acts as a series transmission gain, producing +35.5 dB *expansion* |
| VCA (dbx/SSL) | `EF1.out -> VCA1.cv` | **Virtual** — `vca()` is never stamped into the circuit; `cv` is not a valid modulation sink |
| OTA (Dyna Comp) | `EF1.out -> U1.iabc` | **Weak and nondeterministic** — 0–4.2 dB GR, ratio 1.0–1.34:1, varies run-to-run |

The passive EQ story is better — the WDF core handles the Pultec-style RC/LC
networks and hits analytic frequency-response targets in isolation — but
switched components collapse passive networks to passthrough, and adding any
makeup-gain stage after a passive network flattens its response.

None of this is architecturally fatal. The engine has the right machinery
(dynamic WDF leaves with per-sample resistance updates, a sidechain processor
framework, multi-NL solvers, transformers, inductors). The gaps are in the
**wiring layer between detectors and gain elements**, plus a handful of
compile bugs. The prioritized list is in §6.

## 1. What already works (inventory)

**DSL** (`pedalkernel/src/dsl.rs`): resistors/caps (with dielectric
parasitics)/inductors/pots (3 tapers)/tempco; switched R/C/L and rotary
switches; diodes/zeners; ~100 BJT, ~100 JFET, MOSFET models; 10 op-amp models
incl. CA3080 OTA; ~20 triodes (incl. vari-mu 6386) and ~25 pentodes;
photocouplers (`vtl5c3/vtl5c1/nsl32/t4b`); `envelope_follower(atkR, atkC,
relR, relC, sensR)`; LFO (6 waveforms); VCO/VCF/VCA/comparator/analog switch;
BBD and delay lines; multi-winding transformers with Jiles-Atherton cores and
tertiary feedback windings; `supplies` blocks with sag modeling (±rails, B+
with rectifier impedance); `equipment` as a first-class top-level keyword;
sidechain blocks and rate-divided subcircuits; GR metering (`[gr]` monitors).

**Engine** (`pedalkernel-rt`): SPQR decomposition into WDF trees / multi-NL
R-type stages / IIR / state-space; NR with K-method LUT fast path;
photocoupler with dual-exponential (fast/slow) memory and asymmetric
rise/fall (`elements/controlled.rs:113`); JFET variable resistor with
triode-region Rds law (`elements/controlled.rs:340`); both are true dynamic
WDF leaves (per-sample port-resistance updates with dirty-propagation).
Inductors are textbook one-ports — Pultec-grade passive LC works. Sidechains
run as embedded sub-processors with one-sample feedback delay
(`stage.rs:6375`). Op-amp GBW/slew/rail limits applied as post-processing.

**Test/validation infra**: 41 test files; Goertzel-based `audio_analysis.rs`
(THD, band energy, aliasing, modulation detection); golden binary regression;
`pedalkernel-validate` SPICE harness (pinned ngspice v43 via Docker — not
available in this environment); CI feature matrix.

## 2. Validation baseline (from reports/2026-06-09, post-fix run)

23/42 SPICE comparisons pass. Directly relevant to outboard gear:

- **Transformer step-down fails by ~19 dB** (step-up passes) — every classic
  outboard unit is transformer-coupled; a turns-ratio/scaling bug here
  poisons all line-level I/O modeling.
- **OTA (CA3080) compile-fails** in the rigid MNA fallback ("Unsupported NL
  device kind in general MNA").
- `rc_highpass`, `series_rlc`, `twin_t_notch` still fail — these are EQ
  building blocks.
- Standalone diode/zener fixtures miss by 6–7 dB (zener hard clip ~17 dB) —
  matters for precision-rectifier sidechains.
- **Zero SPICE coverage** for: photocoupler GR loops, envelope follower
  timing, VCA, vari-mu, switched components, pot positions.

## 3. New example circuits and measured behavior

Four examples added under `pedalkernel/examples/outboard/`, each with
researched real-unit values and a "SIMPLIFICATIONS vs real unit" header;
tested by `pedalkernel/tests/outboard_examples.rs` (8 tests, passing — they
assert compile + health and *print* response data rather than asserting
behavior the engine cannot yet produce).

**`compressor/fet_leveler.pedal`** (Urei 1176 rev D pattern: input pot →
series-R/FET-shunt divider → BJT line amp → 1:1 output transformer; feedback
detector with the real 25 kΩ·220 nF attack / 5 MΩ·220 nF release constants).
Measured: loud and quiet program both −21.56 dB — the `vgs` modulation sink
accepts the routing but has **zero effect** on the audio path. Reproduced on
a minimal divider and on the existing `test_pedals/envelope_follower.pedal`
fixture topology, so this is a general engine gap, not a circuit error.

**`compressor/opto_leveler.pedal`** (LA-2A pattern: 100k series / T4B shunt
divider → 12AX7 stage at B+ 250 V; detector drives the LED). Measured: loud
+14.76 dB, quiet −20.72 dB — **+35.5 dB of upward expansion**. The engine
treats LED drive as a series transmission gain irrespective of whether the
LDR is wired series or shunt; in purely passive stages LED modulation has no
effect at all. The T4B element itself (dual time-constant memory) is the
right physics — it is the topology semantics that are wrong.

**`compressor/vca_bus_comp.pedal`** (SSL G-bus/dbx 160 pattern: NE5532
buffers around an SSM2164, feed-forward detector with SSL-derived timing).
Measured: GR delta 0.11 dB ≈ none, as predicted from code: `vca()` is
`GraphRole::Virtual` with no MNA stamp, `compile.rs:308` populates
`vcas: Vec::new()`, and `cv` is not among the valid modulation sinks
(`vgs/gate/led/vgk/vg1k/iabc/clock/speed_mod/delay_time`). Additional compile
bugs found while building it: a standalone **inverting op-amp stage compiles
to silence**, and **DC- or resistor-coupled cascaded op-amp stages compile to
silence** (cap coupling works).

**`eq/passive_lc_eq.pedal`** (Pultec EQP-1A pattern: RC LF boost shelf,
47 mH + 22 n series-resonant HF boost (~5 kHz), RC HF cut; real inductor tap
values 27/33/47/68/82/150 mH preserved in comments). Measured response moves
with the controls (HF boost shifts 10 kHz gain by ~0.9 dB — asserted), but
three engine limits forced deviations: `cap_switched`/`inductor_switched`
**collapse the network to unity passthrough with dead controls**; a pot in
parallel with the inductor does the same; and **every makeup-gain stage tried
(12AX7, JFET buffer, NE5532) either flattened the passive response or
compiled to −84…−240 dB output**, so v1 ships passive-only.

Research note: the real EQP-1A's LF boost/cut sections are RC shelving — the
only inductor is the HF boost. The folklore "10 H tapped inductor" belongs to
other Pultec models (MEQ-5). Sources are cited per-circuit in the file
headers (UA manuals, GroupDIY measurement threads, Gyraf GSSL docs,
ElectroSmash, dbx/THAT datasheets, VPR Alliance 500-series spec).

## 4. New measurement layer (test expansion, delivered)

`tests/audio_analysis.rs` gained a dynamics/EQ characterization layer, and
`tests/measurement_validation.rs` proves it against analytic ground truth:

- `frequency_response_db` — Goertzel sweep, fresh processor per point.
  Validated: RC lowpass measures −3.010 dB at fc (analytic −3.01) and
  −20.5 dB/decade rolloff.
- `static_gain_curve` + `compression_ratio` + `gain_reduction_db` — input
  level sweep with per-point recompile. Validated: op-amp buffer measures
  ratio 1.0000:1; a synthetic 4:1 knee recovers its ratio.
- `tone_burst` + `measure_attack_seconds`/`measure_release_seconds` —
  ±1 dB-settling convention on a 2 ms RMS envelope. Validated against a
  one-pole envelope within ~5%.

These are the tools all future compressor/EQ acceptance tests build on.

**Finding while validating:** `dyna_comp.pedal` compiles to measurably
different processors across processes — flat-linear (ratio 1.0000, GR 0) on
some runs, compressing (ratio 1.34, GR 4.2 dB) on others, suggesting
hash-ordering nondeterminism in compilation. `engine_determinism.rs` covers
in-process determinism only.

## 5. Per-unit readiness matrix

| Target unit | Gain element | Blockers (§6 refs) | After fixes |
|---|---|---|---|
| Urei 1176 | JFET shunt (exists) | G1, G2, G6 | Feasible — best-documented timing values; start here |
| LA-2A | T4B opto (exists, good physics) | G1, G3, G6 | Feasible — tube stage + transformer already work |
| Dyna Comp / Ross (bridge) | OTA (exists) | G5, G9; OTA MNA compile bug | Near-term; smallest gap |
| dbx 160 | Blackmer VCA | G4 + **new RMS detector model** (log-domain 35 ms averager) | Largest new modeling work |
| SSL G-bus | dbx 2150-class VCA | G4, G1, **stereo link** (G10) | After dbx-class VCA exists |
| Pultec EQP-1A | passive + tube makeup | G7, G8, transformer step-down (G6) | Passive section already close |
| API 550A | LC feedback EQ around 2520 | G5 (op-amp coupling/silence), G7, proportional-Q network | Hardest EQ; needs active-feedback EQ support |
| 500-series generally | ±16 V, +4 dBu, balanced I/O | `supplies` syntax exists; no line-level test conventions (G11) | Define +4 dBu/600 Ω test harness conventions |

## 6. Prioritized gaps

**P0 — dynamics control path (blocks every compressor):**
- **G1. Detector taps are fake.** Envelope followers read the *global pedal
  input* at runtime (`processor.rs`: `binding.envelope.process(input)`)
  regardless of the `EF1.in ->` net. Feedback detectors (1176, LA-2A, SSL)
  are unrepresentable; even feed-forward taps at interior nodes are ignored.
- **G2. `-> J1.vgs` envelope routing is inert** for JFET variable resistors
  in the audio path (LFO routing works in Phase 90, so the break is specific
  to how EF bindings reach JfetVariableResistor leaves).
- **G3. Photocoupler topology semantics.** LED drive modulates a series
  transmission gain instead of the LDR's electrical position in the network;
  shunt cells therefore produce expansion. The element model itself is good.
- **G4. VCA is a stub.** Needs either an MNA stamp + `cv` modulation sink,
  or an honest behavioral gain block — currently it parses and does nothing,
  silently.

**P1 — compile correctness (blocks EQs and line amps):**
- **G5. Compile-to-silence bugs:** inverting op-amp stages; DC/resistor
  coupling between op-amp stages; `delay_simple` ("No circuit edges found");
  OTA in rigid MNA fallback.
- **G6. Transformer step-down ~19 dB error** (SPICE-confirmed).
- **G7. Switched components (`cap_switched`/`inductor_switched`) collapse
  passive networks** to passthrough — blocks Pultec/API frequency selection.
- **G8. Stage isolation around passive networks:** any buffer/makeup stage
  after the passive EQ flattens its response and freezes its pots; pots
  absorbed into multi-NL stages accept writes but don't change AC response.

**P2 — fidelity and trust:**
- **G9. Cross-process compile nondeterminism** (dyna_comp evidence above).
- **G10. Stereo/linked sidechains** — engine is mono; bus compressors need a
  shared CV across two channels.
- **G11. Line-level conventions** — define +4 dBu (1.228 V RMS) stimulus,
  600 Ω source/load, ±16 V rails as the standard outboard test setup
  (today's tests assume guitar level into 9 V circuits).
- **G12. RMS detection** — dbx-style log-domain averager (35 ms) as a
  detector option alongside the existing peak-ish RC envelope follower.
- Remaining SPICE failures (rc_highpass, series RLC, twin-T, diode/zener
  fixtures) and zero SPICE coverage for every dynamics element.

**P3 — conveniences:** sidechain HPF idiom, wet/dry parallel mix, ratio
switching via the existing `resistor_switched`, program-dependent release
verification against published T4B data, per-band GR metering.

## 7. Recommended test expansion (next steps)

1. **Lock today's truths:** the measurement layer's analytic tests (done) and
   the printed-measurement tests on the four outboard examples (done) form
   the baseline. As each P0 gap is fixed, promote the corresponding printed
   measurement to a hard assertion (e.g. fet_leveler: GR > 5 dB at 0.5
   amplitude, ratio rising with sidechain drive; opto_leveler: gain *falls*
   with level; attack/release within 2× of the RC design values).
2. **SPICE fixtures for dynamics:** a JFET-shunt GR divider with a stepped
   gate voltage, an opto divider with stepped LED current, and an envelope
   follower timing fixture — these validate the gain-element physics
   independently of the detector wiring.
3. **EQ acceptance curves:** full 20 Hz–20 kHz sweeps of `passive_lc_eq` at
   pot extremes, asserted against transfer functions computed from the
   netlist values (the RC/LC math is closed-form); same pattern later for
   API-style proportional-Q bands.
4. **A determinism test** that compiles the same circuit in N subprocesses
   and compares static gain curves (catches G9).
5. **Line-level harness:** a `+4 dBu / ±16 V` variant of the compile helpers
   so outboard circuits are exercised at realistic levels and headroom.

## 8. Artifacts on this branch

- `pedalkernel/tests/audio_analysis.rs` — measurement layer (additive only)
- `pedalkernel/tests/measurement_validation.rs` — 6 tests, analytic ground truth
- `pedalkernel/examples/outboard/{compressor,eq}/*.pedal` — 4 circuits
- `pedalkernel/tests/outboard_examples.rs` — 8 tests
- This report.
