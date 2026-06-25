# Guitar Pedal Audit — SPICE Conformance & Realtime Readiness

**Date:** 2026-06-25
**Commit:** `5e22c06` (branch `claude/nice-ride-utfvu7`)
**Method:** Fresh `pedalkernel-validate run --suite all` (WDF vs committed ngspice
goldens, 96 kHz / 4× OS) + `realtime_gate` test + `rt_bench` example, all built
`--no-default-features` (no JACK in this env). `xtask` was temporarily dropped
from the workspace to build around a blocked nih-plug git fetch; **reverted, not
committed**.

> Note on terminology: the SPICE suite validates **single-stage cores**
> (`bigmuff_stage`, `fuzz_core`, `fuzz_face_pnp` …), which are *not* the full
> product pedals (`big_muff.pedal`, `fuzz_face.pedal` …). The realtime gate
> measures the **full product pedals**. Both are reported below and kept
> distinct.

---

## TL;DR

- **SPICE conformance: 55/94 signals pass (58.5%).** The headline failure is a
  **systematic BJT common-emitter operating-point gap** — every transistor
  fuzz/gain core is off, `bigmuff_stage` catastrophically so (+70–90 dB).
- **Realtime: the gate is currently RED**, and *not* because of the known
  `big_muff` outlier. **`blues_driver` has regressed to 1.4–1.7× realtime**
  (gate requires ≥2×). `big_muff` remains the documented ~0.3× non-RT outlier.
  Everything else gated clears comfortably (24–136×).
- **Big Muff is the worst pedal on both axes:** golden exists and is sound, but
  WDF output is ~×1000 wrong *and* it doesn't run in realtime.
- **Coverage gap:** ~20 product pedals (SD-1, OCD, Phase 90, CE-2, DM-2, Memory
  Man, the amps, eurorack voices) have **neither** a SPICE golden **nor** an RT
  gate entry.

---

## 1. SPICE conformance — which pedals are "off" vs golden

Failures are split by the test's own **profile** tag, which encodes intent:
`measured_margin`/`strict` = a real gate that *should* pass; `known_gap` =
documented expected miss; `pending` = no trustworthy ngspice golden yet.

### 1a. GENUINE conformance gaps (real, should pass — these are the "off" ones)

| Suite | Test | Profile | RMS err | Notes |
|---|---|---|---|---|
| pedals | **bigmuff_stage** | measured_margin | **+70.2 / +90.2 dB** | **Catastrophic.** BJT common-emitter + collector-diode bias at totally wrong operating point (~×1000). Gate is deliberately negative-threshold so it fails loudly. Golden is sound ngspice. |
| pedals | **fuzz_core** | measured_margin | +7.7 / +26.8 dB | Same BJT family; badly off, not catastrophic. THD 110 dB → harmonics meaningless. |
| active | **fuzz_face_pnp** | measured_margin | +1.1 dB | Level ~right but **22 dB spectral / 35 dB THD** — wrong harmonic structure (PNP germanium bias). |
| active | **npn_common_emitter** | measured_margin | 0.8 / 3.8 dB | Canonical BJT CE stage — 30 dB spectral error. Root cause of the fuzz family. |
| pedals | **rat_clipper** | measured_margin | 3.4 / 5.3 dB | Op-amp + hard clip-to-ground; 40–48 dB spectral. |
| pedals | **ts808_clipper** | measured_margin | 7.6 / 8.1 dB | Op-amp + diode-feedback clip; 13–17 dB spectral. |
| nonlinear | diode_clipper | measured_margin | clipping/sweep fail | Peak +5.4 / +7.0 dB on clipping & sweep. |
| nonlinear | zener_clipper | measured_margin | clipping fail | Peak +2.5 dB. |
| canonical | opamp_diode_clipper | strict | — | Strict gate; diode-feedback op-amp. |
| extraction | sallen_key_lowpass / _highpass | strict | — | Controlled-source filter extraction. |
| extraction | active_feedback_treble_shelf | strict | — | Control-routing regression. |
| extraction | mfb_highpass_controlled | strict | — | Multiple-feedback active filter. |
| reactive | lc_resonant | measured_margin | sweep fail | Resonant point OK, swept response off. |

**Cross-cutting root cause:** `bigmuff_stage`, `fuzz_core`, `fuzz_face_pnp`,
`npn_common_emitter` are all **BJT common-emitter** stages and all fail. This is
one engine gap (transistor operating-point / collector load), not four pedal
bugs. Fixing the CE stage should move all four at once.

### 1b. KNOWN GAPS (documented, expected to fail — not regressions)

| Suite | Tests | Profile |
|---|---|---|
| drums | all 20 (TR-808/909 voices) | known_gap — 0/20, resonator voices not modeled |
| compressor | fet_leveler_level_sweep, fet_leveler_tone_burst | known_gap — 1176-style dynamics baseline gap |

### 1c. PENDING (no trustworthy ngspice golden yet — cannot judge)

| Suite | Tests | Profile |
|---|---|---|
| compressor | la2a_level_sweep, la2a_tone_burst | pending_reference |
| active | ota_ca3080 | pending |

### 1d. Suites fully GREEN

`linear` (4/4), `opamp` (6/6), `eq` Pultec (5/5), `tubes` (6/6 — triodes,
pentode, vari-mu all clean), `tape` (J-A head, THD err ~0.0 dB), `stress`.
Tubes and tape are the most faithful parts of the engine today.

> The `legends` goldens (goldenrod, ratking, screamer, sd1) are **not** part of
> the SPICE suite — they back separate hardware-reference tests
> (`screamer_drive_sweep`, `hardware-goldens/`) and were not exercised here.

---

## 2. Realtime readiness — which pedals are / aren't RT-ready

Gate (`tests/realtime_gate.rs`): compile + process at 48 kHz, require **≥2×
realtime** (≤50% of one core). Measured in this container (release,
`--no-default-features`):

| Pedal | ×realtime | Status |
|---|---|---|
| fuzz_face | 114–136× | ✅ ample |
| dyna_comp | ~58× (rt_bench) | ✅ |
| proco_rat | 30–37× | ✅ |
| tube_screamer | 37× | ✅ |
| klon_centaur | 17–24× | ✅ |
| **blues_driver** | **1.4–1.7×** | ❌ **FAILS 2× gate** — ~13,000 ns/sample, ~20× slower than peers. Gate is currently RED on this. Structural (consistent across runs), not runner noise. |
| **big_muff** | **~0.3×** (documented) | ❌ not RT-ready — 4 coupled NL stages; observe-only in the gate, tracked for a perf sprint. |

Other harnessed circuits (`rt_bench`), all RT-ready: opto_leveler 16×,
fet_leveler 32×, cem3340_vco 40×, minisynth 15×, space_echo 37×, spring_reverb
21–23×.

**Two pedals are not realtime-ready: `big_muff` (~0.3×) and `blues_driver`
(~1.5×).** The gate aborts at `blues_driver`, so `dyna_comp`/`big_muff` don't
print in a gate run — `big_muff`'s figure is the documented nightly value.
`blues_driver` dropping under 2× is the actionable regression: it was expected
to pass.

---

## 3. Coverage gaps (no golden AND/OR no RT measurement)

Product pedals with **neither** a SPICE golden **nor** an RT gate entry — i.e.
unaudited on both axes:

- Overdrive/dist: **sd1**, **fulltone_ocd** (klon/rat/TS/blues are gated)
- Modulation/delay: **phase90**, **boss_ce2**, **boss_dm2**, **memory_man**
- Amps: **marshall_jtm45**, **tweed_deluxe_5e3(_full)**, **bassman_5f6a**
- Eurorack/synth: **kick_808**, **snare_808**, **moog_ladder_vcf**
  (cem3340_vco + minisynth are in rt_bench)
- Outboard: **vca_bus_comp**, **passive_lc_eq** (la2a/opto/fet are partial)

(SD-1 *does* have a `legends/sd1` hardware golden, but it's outside the SPICE
suite, so its WDF-vs-reference delta is not gated in CI.)

---

## 4. Recommended priorities

1. **BJT common-emitter operating point** (one fix, four+ tests): unblocks
   `bigmuff_stage`, `fuzz_core`, `fuzz_face_pnp`, `npn_common_emitter`, and the
   `big_muff` product pedal's correctness. Highest leverage.
2. **`blues_driver` RT regression** to ≥2×: the gate is red. Profile the
   ~13 µs/sample hot path (likely controlled-resistor scattering re-derivation
   or an over-large MultiNl/Newton group).
3. **`big_muff` perf sprint**: the 4-coupled-NL-stage ~0.3× outlier — needs
   K-method tabulation of the coupled group, not a gate exception.
4. **Close coverage gaps**: add ngspice goldens + RT-gate entries for SD-1,
   OCD, and the amps so they stop being unaudited.

---

*Generated from a live validation run; profiles and error figures are quoted
verbatim from `pedalkernel-validate run --suite all`. No source changed; the
temporary `xtask` workspace exclusion was reverted.*
