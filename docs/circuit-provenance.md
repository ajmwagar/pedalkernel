---
title: "Circuit provenance"
description: "Accuracy-stamp registry: what each .pedal circuit claims to model, where the schematic came from, how verified the values are, and what the engine currently does with it."
section: "Reference"
weight: 55
---

# Circuit provenance registry

Every `.pedal` file in the repo, stamped for provenance. Generated 2026-06-12
from file headers, `git log` dates, and the 2026-06-12 outboard/Eurorack audit
(`reports/outboard-gear-audit-2026-06-12.md`,
`reports/outboard-fix-and-product-plan-2026-06-12.md`).

## Stamp semantics

- **[V] Verified** — the header cites a schematic-level source AND the
  component values were cross-checked against that source (either during the
  2026-06-12 research session, or at the file's own documented verification
  date). Several [V] files additionally flag *individual* values as `[V]`
  (verified) or `[A]` (approximate/adapted) inline — the Eurorack headers are
  the exemplar.
- **[C] Cited** — the header names a source (e.g. "ElectroSmash analysis",
  "R.G. Keen"), but the values have not been independently re-checked this
  session. Last-checked = not re-verified; the git date is the best bound.
- **[U] Uncited** — the header describes the modeled unit but names no
  schematic source. Needs provenance work before any accuracy claim.

**No URLs are invented here.** Where a header names a source without a link,
the Source column says so ("name only"). The only URLs reproduced below are
ones that literally appear in file headers.

**Engine status is a separate axis from provenance.** A circuit can be
perfectly transcribed from a verified schematic and still be silent in the
current engine (or vice versa). Engine-status entries reference the F-numbered
fixes in `reports/outboard-fix-and-product-plan-2026-06-12.md`; blank means no
known engine defect specific to that circuit as of 2026-06-12.

## Going-forward convention (proposed)

Every new `.pedal` header should carry a `PROVENANCE` block:

- the real unit modeled (manufacturer, model, revision, year),
- named sources **with URLs** where they exist,
- per-value verification flags (`[V]` verified against source / `[A]`
  approximate or adapted),
- a `SIMPLIFICATIONS` list of intentional deviations from the real unit,
- a verified-on date.

`pedalkernel/examples/outboard/compressor/fet_leveler.pedal` and
`pedalkernel/examples/eurorack/drums/kick_808.pedal` are the exemplars —
copy their structure.

---

## Shipped product pedals (`pedalkernel/examples/pedals/`)

All git-dated 2026-04-22.

| Circuit | Models | Source (as cited in header) | Stamp | Last checked | Engine status (2026-06-12) |
|---|---|---|---|---|---|
| `overdrive/tube_screamer.pedal` | Ibanez TS-808 (1979), JRC4558D | none named | [U] | not re-verified; git 2026-04-22 | Working (golden); compiles a load-bearing feedforward stage — F2 ships with golden re-bless |
| `overdrive/klon_centaur.pedal` | Klon Centaur (1994), TL072 + Ge diodes | none named (Bill Finnegan named as designer, no schematic source) | [U] | not re-verified; git 2026-04-22 | Working (golden); F2 re-bless caveat as above |
| `overdrive/sd1.pedal` | Boss SD-1 (1981), JRC4558 asymmetric clip | "Original Boss SD-1 schematic (MIT), R.G. Keen's analysis" — names only | [C] | not re-verified; git 2026-04-22 | — |
| `overdrive/blues_driver.pedal` | Boss BD-2 (1995), FET + op-amp | none named | [U] | not re-verified; git 2026-04-22 | Working (golden); F2 re-bless caveat |
| `overdrive/fulltone_ocd.pedal` | Fulltone OCD V1.7, 2N7000 MOSFET clip | none named | [U] | not re-verified; git 2026-04-22 | **Compile error** — MOSFET/Zener unsupported in multi-NL MNA, pending F12 (fix in flight) |
| `distortion/proco_rat.pedal` | ProCo RAT (1988 "Whiteface" reissue), LM308 | none named in this file (the ElectroSmash-verified BOM lives in the root `ratking_*` files — port it) | [U] | not re-verified; git 2026-04-22 | Working (golden); F2 re-bless caveat |
| `fuzz/fuzz_face.pedal` | Dallas Arbiter Fuzz Face (1966), AC128 PNP Ge | ElectroSmash analysis + R.G. Keen "Technology of the Fuzz Face" — names only; full real BOM in header | [C] | not re-verified; git 2026-04-22 | — |
| `fuzz/big_muff.pedal` | EHX Big Muff Pi "Ram's Head" (~1973) | none named | [U] | not re-verified; git 2026-04-22 | — |
| `compressor/dyna_comp.pedal` | MXR Dyna Comp, CA3080 OTA | none named | [U] | not re-verified; git 2026-04-22 | Compiles nondeterministically across processes (F1, in flight); OTA stage lowers to a flat-linear Passthrough — deterministic-but-flat pending F9 |
| `delay/boss_dm2.pedal` | Boss DM-2, MN3005 BBD | none named | [U] | not re-verified; git 2026-04-22 | **Silent** — BBD components never lowered, pending F11 (fix in flight) |
| `delay/memory_man.pedal` | EHX Deluxe Memory Man, MN3005 BBD | none named | [U] | not re-verified; git 2026-04-22 | **Silent** — pending F11 |
| `modulation/boss_ce2.pedal` | Boss CE-2, MN3207 BBD chorus | none named | [U] | not re-verified; git 2026-04-22 | **Silent** — pending F11; stage[2] also hit by F13c passive-stage hard-zero |
| `phaser/phase90.pedal` | MXR Phase 90 Script Logo (1974), 4-stage JFET | "Based on the AURORA implementation" — name only | [C] | not re-verified; git 2026-04-22 | Recovered by the F2 fix (2026-06-12) |

## Amps (`pedalkernel/examples/amps/`)

All git-dated 2026-04-22. Headers are rich on circuit topology and history but
name no schematic source (no layout document, no published schematic cited).

| Circuit | Models | Source (as cited in header) | Stamp | Last checked | Engine status (2026-06-12) |
|---|---|---|---|---|---|
| `bassman_5f6a.pedal` | Fender Bassman 5F6-A (1958–60), preamp + TMB stack | none named | [U] | not re-verified; git 2026-04-22 | — |
| `marshall_jtm45.pedal` | Marshall JTM45 / Bluesbreaker (1962–66), preamp | none named | [U] | not re-verified; git 2026-04-22 | — |
| `tweed_deluxe_5e3.pedal` | Fender Tweed Deluxe 5E3 (1958) | none named | [U] | not re-verified; git 2026-04-22 | LSB-scale (~1e-13) compile nondeterminism, pending F14 — inaudible but breaks bit-exactness |
| `tweed_deluxe_5e3_full.pedal` | Fender Tweed Deluxe 5E3 (1958), full incl. PI/6V6/5Y3 | none named | [U] | not re-verified; git 2026-04-22 | Same F14 nondeterminism |

## Synths (`pedalkernel/examples/synths/`)

All git-dated 2026-04-22.

| Circuit | Models | Source (as cited in header) | Stamp | Last checked | Engine status (2026-06-12) |
|---|---|---|---|---|---|
| `cem3340_vco.pedal` | Curtis CEM3340 / AS3340 VCO module | none named (chip + host units named, no datasheet/app-note cited) | [U] | not re-verified; git 2026-04-22 | **Silent** — pending F13 / VCO lowering |
| `minisynth.pedal` | AS3340 + AS3320 + V2164 monosynth voice | none named (chips named, no sources) | [U] | not re-verified; git 2026-04-22 | **Silent** — pending F13 / VCO lowering |
| `moog_ladder_vcf.pedal` | Minimoog 4-pole transistor ladder (discrete) | none named | [U] | not re-verified; git 2026-04-22 | **Silent** — passive-stage hard-zero, pending F13c |

## Outboard studio gear (`pedalkernel/examples/outboard/`)

Researched and written 2026-06-12; all carry full `PROVENANCE` +
`SIMPLIFICATIONS` blocks with measured engine evidence. Headers cite the real
unit at revision level; the audit report records the source families used
(UA manuals, GroupDIY measurement threads, Gyraf GSSL docs, dbx/THAT
datasheets) — **names only, no URLs were recorded in these headers**.

| Circuit | Models | Source (as cited in header) | Stamp | Last checked | Engine status (2026-06-12) |
|---|---|---|---|---|---|
| `compressor/fet_leveler.pedal` | Urei 1176LN rev D "blackface" (1970) | unit-level provenance block (topology, time constants, ratios); source family per audit report — names only | [V] | 2026-06-12 | Envelope→JFET path live (F5 fixed) but feedback detector starved: ~0 dB GR until makeup-gain (G5) and transformer ~19 dB defects are fixed |
| `compressor/opto_leveler.pedal` | Teletronix LA-2A (~1962) | unit-level provenance block (T4B time constants) — names only | [V] | 2026-06-12 | **Wrong-direction dynamics** — photocoupler modeled as series gain, produces expansion not compression, pending F6; also F14 LSB nondeterminism |
| `compressor/vca_bus_comp.pedal` | SSL 4000 G bus comp / dbx 160 hybrid | unit-level provenance block (dbx 202C/2180, SSL auto-release RC pairs, dbx RMS detector) — names only | [V] | 2026-06-12 | **No compression** — `vca()` is an unstamped stub (`vcas: Vec::new()`), pending F8; inverting op-amp stages compile to silence (F3) |
| `eq/passive_lc_eq.pedal` | Pultec EQP-1A (~1961), passive section | unit-level provenance block (real frequencies, real inductor taps 27–150 mH) — names only | [V] | 2026-06-12 | Works passive-only with responsive controls; every makeup stage flattens/kills the network (F10), switched components collapse to passthrough (F7) |

## Eurorack (`pedalkernel/examples/eurorack/`)

Researched and written 2026-06-12 with per-value `[V]`/`[A]` flags — the
exemplar provenance format. Drums committed 2026-06-12; `acid/tb303_vcf.pedal`
is **untracked** (created this session, no git date yet; owned by another agent).

| Circuit | Models | Source (as cited in header) | Stamp | Last checked | Engine status (2026-06-12) |
|---|---|---|---|---|---|
| `acid/tb303_vcf.pedal` | Roland TB-303 diode-ladder VCF | T. Stinchcombe filter papers (`timstinchcombe.co.uk`); x0xb0x BOM (`ladyada.net`); WimOliphant KiCad recreation — name only; Roland TB-303 service notes — name only | [V] | 2026-06-12 | Gated by F13 (Eurorack line); knob direction inverted vs hardware (documented); top common-base pair approximated as a fourth rung |
| `drums/kick_808.pedal` | Roland TR-808 bass drum voice | Werner/Smith/Abel DAFx-14 paper — name only; Werner PhD thesis (`purl.stanford.edu/jy057cz8322`); Roland TR-808 service notes — name only; Tiptop BD808 manual — name only | [V] | 2026-06-12 | Resonator works; decay-regeneration feedback unbuildable (every wiring breaks the engine — F13a family), ships at hardware minimum-decay; trigger needs series R workaround |
| `drums/snare_808.pedal` | Roland TR-808 snare (tonal path) | Werner TR-808 analysis series — name only; Werner PhD thesis (`purl.stanford.edu/jy057cz8322`); Roland TR-808 service notes — name only; Tiptop SD808 manual — name only | [V] | 2026-06-12 | Parallel resonator branches don't sum, blend pot inert (F13b); injection point moved vs hardware (documented); no noise "snappy" path (no noise primitive in DSL) |

## Root-level (`ratking_*.pedal`)

The RAT product line's working files. Both git-dated 2026-04-22.

| Circuit | Models | Source (as cited in header) | Stamp | Last checked | Engine status (2026-06-12) |
|---|---|---|---|---|---|
| `ratking_non_invert.pedal` | ProCo RAT (1978 / 1988 Whiteface), v5 non-inverting rewrite | "ElectroSmash verified schematic" — name only; full real BOM transcribed in header with derived pole frequencies | [V] | 2026-04-22 (v5 rewrite; ElectroSmash BOM cross-check documented in header) | — |
| `ratking_non_invert_v1a.pedal` | Same, v5a variant | same as above | [V] | 2026-04-22 | JFET output buffer deliberately omitted — incomplete JFET Rds modeling kills the signal (documented in header); restore when JFET modeling is solid |

## Test fixtures (`pedalkernel/tests/test_pedals/`) — summarized

64 files, git-dated 2026-04-22. These are overwhelmingly **generic component
fixtures**, not product models: BJT part sweeps (`bjt_2n2222` … `bjt_nkt275`),
triode/pentode part sweeps (15 pentodes, 10 triodes), clipping variants
(`clip_*`), cap models (`cap_da`, `cap_leaky`), LFO waveforms, op-amp
buffer/gain/slew, BBD timing fixtures, photocouplers (`photocoupler_t4b`,
`photocoupler_vtl5c3`), `bridged_t_resonator`, `envelope_follower`,
`jfet_allpass`/`jfet_switch`, `ota_ca3080`. They reference real *part numbers*
(datasheet-level provenance is implicit in the component models, which is the
engine's concern, not these netlists') and need no per-circuit provenance.
Four reference real products and are itemized:

| Circuit | Models | Source (as cited in header) | Stamp | Last checked | Engine status (2026-06-12) |
|---|---|---|---|---|---|
| `klon_nl_block.pedal` | Klon Centaur Ge clipping block (isolated) | none named | [U] | not re-verified; git 2026-04-22 | — |
| `rat_nl_block.pedal` | ProCo RAT diode clipping block (isolated) | none named | [U] | not re-verified; git 2026-04-22 | — |
| `walrus_slo.pedal` | "Walrus Audio Slo-style" multi-tap BBD (explicitly -style, not a transcription) | none named | [U] | not re-verified; git 2026-04-22 | **Silent** — pending F11 |
| `ehx_memory_man.pedal` | "EHX Deluxe Memory Man-style" BBD test (explicitly -style) | none named | [U] | not re-verified; git 2026-04-22 | Passes **dry leakage only** — pending F11 |

## SPICE validation circuits (`pedalkernel-validate/circuits/`) — summarized

50 files, git-dated 2026-06-03 and earlier. These exist to validate the engine
against ngspice, not to model products: canonical filters (`rc_lowpass`,
`series_rlc`, `twin_t_notch`…), op-amp configurations, device-level
nonlinears, transformers, and textbook active stages. The `pedals/` subfolder
(`bigmuff_stage`, `fuzz_core`, `rat_clipper`, `ts808_clipper`) and a few
`active/` files (`fuzz_face_pnp`, `ota_ca3080`, `push_pull_6l6`,
`common_cathode_12ax7`) reference real-product *sub-blocks* but are
single-stage SPICE fixtures, not full-product models — no per-circuit
provenance needed beyond the SPICE harness itself. Two model a complete real
product and are itemized:

| Circuit | Models | Source (as cited in header) | Stamp | Last checked | Engine status (2026-06-12) |
|---|---|---|---|---|---|
| `eq/pultec_eqp1a.pedal` | Pultec EQP-1A, full circuit-accurate reference (transformers, selectors, 12AX7/12AU7 makeup) | none named — header targets "published curves" generically and itself warns "verify against an original schematic before manufacture" | [U] | not re-verified; git 2026-06-03 | Design-target file: test asserts **parse only**; compile/response checks `#[ignore]`d until the engine renders multi-reactive passive networks (F7/F10/transformer work) |
| `eq/pultec_eqp1a_rlc.pedal` | Pultec EQP-1A, simplified RLC reduction | none named (derives from the file above) | [U] | not re-verified; git 2026-06-03 | Compiles; frequency-response assertions `#[ignore]`d — WDF passive RLC magnitude responses not yet accurate |

---

## Stamp totals (35 itemized circuits; 108 further fixtures summarized)

- **[V] Verified: 9** — 4 outboard, 3 Eurorack, 2 ratking
- **[C] Cited: 3** — fuzz_face, sd1, phase90
- **[U] Uncited: 23** — 10 product pedals, 4 amps, 3 synths, 2 Pultec
  validation references, 4 product-named test fixtures

## Needs provenance work (ranked by product importance)

1. `examples/pedals/overdrive/klon_centaur.pedal` — flagship "Pro" pedal,
   zero cited sources.
2. `examples/pedals/overdrive/tube_screamer.pedal` — the most-cloned circuit
   in the repo's genre, no source named.
3. `examples/pedals/distortion/proco_rat.pedal` — easiest win: port the
   ElectroSmash-verified BOM + citation already present in the root
   `ratking_non_invert.pedal` header.
4. `examples/pedals/compressor/dyna_comp.pedal` — F9's acceptance criterion is
   "ratio/GR matching ElectroSmash" — that source should be in the header.
5. `examples/pedals/fuzz/big_muff.pedal` — header documents careful v2 fixes
   but no schematic source for the values being fixed.
6. `examples/pedals/delay/boss_dm2.pedal`, `delay/memory_man.pedal`,
   `modulation/boss_ce2.pedal` — currently silent (F11); when re-blessed
   post-fix they will need sources to verify against.
7. `examples/pedals/overdrive/blues_driver.pedal`,
   `overdrive/fulltone_ocd.pedal` — no sources.
8. `examples/amps/*` (4 files) — detailed topology notes, no schematic
   citations; tube amp schematics are widely published, low-effort fix.
9. `examples/synths/*` (3 files) — CEM3340 datasheet/app-note and Minimoog
   service schematics are canonical and citable.
10. `pedalkernel-validate/circuits/eq/pultec_eqp1a.pedal` (+ `_rlc`) — the
    file itself asks for schematic verification; the 2026-06-12 audit research
    (real EQP-1A LF sections are RC; the only inductor is the HF boost) should
    be folded into its header.
11. `tests/test_pedals/{klon_nl_block,rat_nl_block,walrus_slo,ehx_memory_man}.pedal`
    — lowest priority; consider renaming to drop product names or adding
    one-line citations.
