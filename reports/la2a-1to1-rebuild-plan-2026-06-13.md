# LA-2A 1:1 Rebuild Plan — earning a true [V] stamp

**Date:** 2026-06-13
**Author:** orchestrator research pass (read-only on code)
**Scope:** Design + gap analysis to rebuild `examples/outboard/compressor/opto_leveler.pedal`
**1:1 with the real Teletronix/UA LA-2A**, no simplifications, so it can earn the
tightened **[V]** stamp = topology-faithful + NO SIMPLIFICATIONS + behavior-validated.

Legend: **[V]** = schematic/manual-verified per a cited source · **[A]** = asserted /
engineering inference (plausible, not pinned to a primary source).

---

## Executive Summary (read this first)

1. The current `opto_leveler.pedal` is LA-2A-*inspired* and lists 8 SIMPLIFICATIONS in
   its own header — it cannot earn the tightened [V].
2. The real LA-2A has **no envelope_follower**. The T4B cell (EL panel + 2 CdS photocells)
   *is* both the detector and the entire timing network; our model fakes that detector
   with an RC `envelope_follower`. **This is the crux gap.**
3. The detector is a real rectified **sidechain circuit** (12AX7 sidechain amp -> 6AQ5
   power tube -> EL panel) tapped off the OUTPUT — feedback. The engine has no way to
   drive a photocoupler LED from a circuit node's rectified signal; the LED sink is bound
   only to an `envelope_follower` element (`bind.rs:716-762`).
4. The T4B's program-dependent release (heavier/longer GR -> SLOWER release) is **not in
   the element physics**: `t4b()` uses fixed asymmetric taus + a constant `slow_weight`
   (`controlled.rs:85-94, 166-205`); release does not depend on accumulated GR history. Our
   current model even measures it BACKWARDS (heavy 1.49 s < light 2.11 s) — that's the
   RC-envelope shortcut, but the element itself also lacks true memory.
5. The Gain pot is mis-placed/mis-coupled: it straddles the WDF GR stage and the MultiNL
   tube-grid stage and is inert (`gain_pot_raises_steady_gain` is `#[ignore]`d). In the
   real unit Gain is post-detector **makeup**, so fixing placement also kills the 1.1 s
   attack regression.
6. Tube stages (12AX7 x2, 12BH7) map cleanly to Koren triode models — low risk.
7. Transformers are the second risk: the engine has J-A cores but `TransformerNode(secondary, n)`
   with `l_secondary = l_primary/n²` (`transformer.rs:86-113`) carries the known ~19 dB
   step-down scaling debt — the output transformer must be validated against SPICE.
8. **Verdict:** [V] is achievable but requires 3 engine fixes first (LED-from-circuit
   sidechain, T4B GR-history memory, transformer step-down) plus a Gain-placement/topology
   fix, THEN the faithful .pedal. Phased plan in section 4.

---

## 1. The real LA-2A circuit, stage by stage

Tube complement: **2x 12AX7A, 1x 12BH7A, 1x 6AQ5** [V] (UA/Teletronix manual; multiple
sources confirm). B+ ~**275 VDC**, tubes fed via separate **220 kΩ** droppers; 12AX7 plate
~**105 V**, 6AQ5 screen ~**100 V** [V] (GroupDIY build measurements). EL panel driven to
**no more than ~90 VAC peak** [V] (AudioScape / Kenetek T4B notes).

### Audio (forward) path
1. **Input transformer** — 600 Ω bridging input, balanced -> unbalanced, into the GR network. [V]
2. **T4B optical attenuator (gain reduction, BEFORE the gain stage).** The audio CdS
   photocell (Clairex CL-505L class) sits as the **SHUNT leg of a passive divider**: a
   series resistor works against the cell to ground. Dark = high R (little loss); lit =
   low R (heavy attenuation). Gain reduction happens *passively, ahead of* the tube. [V]
   (crochambeau walkthrough; GroupDIY T4 thread; AudioScape.)
3. **12AX7 two cascaded gain stages** — makeup voltage amplification after the loss. [V]
4. **Gain pot** — output/makeup level control, positioned **after/between the amplifier**,
   NOT in the detector path. It can only scale makeup; it must NOT affect attack. [V/A]
   (manual describes Gain as the output level control; exact node is [A].)
5. **12BH7A stacked cathode follower** — low-Z driver for the output transformer. [V]
6. **Output transformer** — UTC A-24-style; impedance match + balanced output. [V]

### Sidechain (detector) path — the part with no separate RC
7. Signal is tapped **after the opto cell** (feedback) and fed to a **12AX7 sidechain
   amplifier**, then a **6AQ5 power tube**, which provides the AC drive to **illuminate
   the EL panel** (~90 VAC peak region). [V] (manual: "6AQ5 provides the signal necessary
   to drive the electro-luminescent panel"; analogvibes; KVR/GroupDIY.)
8. **The EL panel + 2 CdS cells ARE the detector AND the timing.** No discrete RC sets
   attack/release. EL turn-on has a soft threshold (panel only glows above some amplitude);
   the CdS material's photoconductive lag sets the dynamics. [V] (AudioTechnology;
   AudioScape; Sound-on-Sound style descriptions.)
   - **Attack ~10 ms.** [V]
   - **Two-stage, program-dependent release:** fast first phase ~**40-80 ms** (≈first 50%
     of recovery), then a slow tail **0.5 s up to ~5 s**, history-dependent —
     **heavier / longer GR -> SLOWER release.** [V] (UA manual + T4B vendor notes;
     attack & release are co-dependent per cell.)
9. **Peak Reduction pot** — scales the sidechain **drive into the EL panel** (more drive ->
   brighter panel -> lower CdS R -> more GR). It is the threshold/depth control. [V]
10. **Limit / Compress switch** — changes sidechain feedback: **Compress** = sidechain is
    ~100% of output (feedback); **Limit** = sidechain ≈ **1/25 input + 24/25 output**
    (mostly feedback with a feed-forward trickle), giving more aggressive action. [V]
    (KVR "Help understanding the LA-2A"; GroupDIY.)
11. **R37 HF pre-emphasis** — sidechain frequency-response trim; turning toward "more highs"
    reduces LF sensitivity of the detector. [V] (UA manual calibration page; GroupDIY.)

---

## 2. Gap table — faithful element vs engine capability vs work needed

| # | Faithful element | Engine today | Gap / verdict | File:line |
|---|---|---|---|---|
| A | Input transformer (600 Ω bridging) | `transformer` element, J-A core, `TransformerNode(secondary, n)` | Topology expressible. Step-up/down scaling must be SPICE-validated (see F). | `transformer.rs:86-113` |
| B | T4B as passive SHUNT divider (R series / CdS to gnd) | `photocoupler(t4b)` as controlled-R edge, shunt to gnd | OK 1:1 as a passive divider leg; LDR compiles as controlled-resistance MNA child (per .pedal header item 6). | `modulation.rs:194-289` |
| C | 12AX7 x2 cascaded gain | Koren `triode(12ax7)` | OK. Need TWO stages, not one (current model has one). Low risk. | `elements/mod.rs` (triode) |
| D | 12BH7 cathode follower | Koren triode; stacked CF topology | Expressible as a triode cathode-follower stage. [A] verify 12BH7 params exist; if not, add model. | `elements/mod.rs` |
| E | Gain pot = post-detector makeup | 3-terminal pot split (`Gain__aw/__wb`) straddling WDF GR + MultiNL tube-grid; inert | **Must relocate** Gain to a pure makeup node (after the tube, before output xfmr) so it is a plain divider, not swallowed into an active group. Fixes inertness AND the 1.1 s attack regression. | `bind.rs` pot routing; red test `la2a_sweeps.rs:50-80` |
| F | Output transformer (UTC A-24) | `TransformerNode`, `l_secondary=l_primary/n²` | **RISK:** known ~19 dB step-down scaling debt (CLAUDE.md outboard audit). Must SPICE-validate level through the output xfmr before [V]. | `transformer.rs:88-90` |
| G | **T4B detector = rectified SIDECHAIN circuit (12AX7+6AQ5+rectifier) from OUTPUT tap driving EL panel** | LED sink bound ONLY to an `envelope_follower` RC element; no path to drive LED from a circuit node's rectified signal | **CRUX GAP.** Need a "detector-from-node" capability: rectify a chosen circuit node (the 6AQ5 plate / EL-drive node) and feed it to `PhotocouplerLed`, replacing the RC envelope_follower. The sidechain amp + 6AQ5 should be REAL tube stages, with the rectifier feeding `PC1.led`. | `bind.rs:494-619` (tap resolution), `bind.rs:716-762` (LED sink only from EF) |
| H | T4B program-dependent timing (attack ~10 ms; release SLOWER after heavier/longer GR) | `t4b()` fixed asymmetric taus + constant `slow_weight`; release does NOT depend on GR history | **GAP.** The dual-state blend exists but `slow_weight` and taus are constants -> no memory. Need GR-history-dependent release (e.g. slow-state weight or tau that increases with accumulated illumination/dwell). Current model measures program-dependence BACKWARDS. | `controlled.rs:85-94` (t4b consts), `controlled.rs:166-205` (`set_led_drive`) |
| I | Peak Reduction = sidechain-drive control | `controls` bind pot positions only; EF sensitivity is a fixed param | Once G lands (real sidechain), Peak Reduction becomes a real pot/trim scaling the sidechain drive node — bindable. Until then it cannot be honestly expressed. | `bind.rs` control resolution; .pedal header item 5 |
| J | Limit/Compress switch (feedback mix) | switched-resistor primitives exist | Expressible as a switch changing the sidechain tap mix (100% output vs 1/25 in + 24/25 out) once G provides a real sidechain node. | n/a (DSL switch) |
| K | R37 HF pre-emphasis | RC in sidechain | Trivial once G provides a real sidechain signal path (an RC before the rectifier). | n/a |

---

## 3. [V] acceptance criteria for the LA-2A

**Topology (1:1, no SIMPLIFICATIONS):**
- Input transformer present (600 Ω bridging). [A] level validated vs SPICE.
- T4B audio cell is the SHUNT leg of a passive divider ahead of the tube. [V]
- TWO cascaded 12AX7 gain stages (not one). [V]
- Gain pot is a post-amplifier MAKEUP divider, provably outside the detector path. [V]
- 12BH7 stacked cathode follower present. [V]
- Output transformer present and level-accurate (within SPICE tolerance, NOT 19 dB off). [V]
- Detector is the **real rectified sidechain** (12AX7 -> 6AQ5 -> rectifier -> EL panel),
  tapped off the OUTPUT (feedback). **No `envelope_follower` anywhere.** [V]
- Peak Reduction scales sidechain drive; Limit/Compress switch changes the tap mix;
  R37 HF pre-emphasis present. [V]

**Measured behavior (validated in tests/):**
- **Attack ~10 ms** (not 1.1 s, not 2 ms) — measured step-attack to 63% of final GR.
- **Program-dependent release that SLOWS after heavier/longer GR:** heavy-GR release
  time > light-GR release time (currently FALSE — must flip), with the fast (~40-80 ms)
  + slow (0.5-5 s) two-phase shape visible.
- **GR curve vs level** matches published LA-2A character: smooth, soft-knee, deep
  leveling from low levels, limiter-grade in the top region (current model ~32.9 dB GR,
  ~15:1 top — keep, but re-validate against a published LA-2A GR curve).
- **Gain pot raises steady output monotonically** (currently `#[ignore]`d / inert).
- **Peak Reduction increases GR depth monotonically** (currently inexpressible).

---

## 4. Phased build plan (engine gaps first, then the faithful .pedal)

**Phase 0 — Spike / decide (human topology decision).** The Gain-pot-straddling-an-
active-group problem (gap E, same family as F10 link-2) needs a topology ruling. Decide:
relocate Gain to a pure passive makeup node (recommended — also fixes attack), and confirm
the detector-from-node mechanism (gap G) design.

**Phase 1 — Engine: detector-from-circuit-sidechain (gap G, CRUX).** Add the ability to
drive `PhotocouplerLed` from a rectified circuit NODE (the 6AQ5/EL-drive node), not an
`envelope_follower`. Build the sidechain as real tube stages feeding a rectifier into the
LED sink. Touch `bind.rs:716-762` (LED binding) and the tap/source resolution at
`bind.rs:494-619`. **Blocks everything downstream.**

**Phase 2 — Engine: T4B GR-history memory (gap H).** Make `t4b()` release depend on
accumulated illumination/dwell so heavier/longer GR -> slower release. Modify
`controlled.rs:85-94` (model) and `set_led_drive` `controlled.rs:166-205` (e.g. let the
slow-state weight or slow tau grow with sustained drive). Validate against the
two-phase 40-80 ms / 0.5-5 s spec. Can run in parallel with Phase 1.

**Phase 3 — Engine: output transformer step-down (gap F).** Diagnose and fix the ~19 dB
scaling in `TransformerNode` (`transformer.rs:86-113`) against the SPICE harness
(`pedalkernel-validate/circuits/reactive/transformer_stepdown.pedal`). Can run in parallel.

**Phase 4 — Engine: Gain placement + Peak Reduction binding (gaps E, I, J, K).** After the
topology ruling, make the makeup pot live and make the sidechain-drive control + switch +
R37 bindable. Depends on Phase 0 + Phase 1.

**Phase 5 — Faithful `.pedal`.** Rebuild with: input xfmr, R-series + T4B shunt divider,
2x 12AX7, Gain makeup pot, 12BH7 CF, output xfmr, real sidechain (12AX7 -> 6AQ5 ->
rectifier -> PC1.led) tapped off output, Peak Reduction, Limit/Compress, R37. Delete the
`envelope_follower`. Depends on Phases 1-4.

**Phase 6 — Validate + stamp.** Promote the `#[ignore]`d tests; add attack-~10 ms,
program-dependent-release-slows, Peak-Reduction-depth tests; SPICE-check transformer
levels; compare GR curve to a published LA-2A. Then claim [V].

---

## 5. Open questions for review

1. **Detector-from-node mechanism (gap G):** new DSL/engine primitive (a "rectifier ->
   modulation-source" element reading a node) vs special-casing the 6AQ5 plate node? This
   is the biggest design decision and should be settled before any code.
2. **T4B memory model (gap H):** what is the physically-honest state variable for GR
   history — accumulated charge on the EL cap, CdS "trap" occupancy, or an empirical
   dwell-weighted slow tau? Need a target reference curve to fit.
3. **Gain pot placement (gap E):** confirm from the schematic the EXACT node (between the
   two 12AX7 stages, after the second, or after the 12BH7). Sources call it the output
   level control [V] but the precise node is [A]. A schematic read is needed for true 1:1.
4. **Limit vs Compress exact mix:** the 1/25 + 24/25 figure is from a forum [V-ish]; pin
   it to the schematic resistor values before stamping the switch.
5. **Two CdS cells:** the second cell drives the meter, not audio. Do we model only the
   audio cell, or both (meter is cosmetic)? Modeling one is faithful to the audio path.
6. **Transformer realism (gap F):** is a full level-accurate output transformer required
   for [V], or is the J-A core acceptable once the step-down scaling is fixed? Owner call.
7. **Component values:** plate/cathode/dropper values are from GroupDIY measurements [V]
   but R37, EL-cap, and the exact divider series-R were not pinned in this pass — a direct
   schematic read (LOC PDF — WebFetch was 403; needs manual retrieval) would close these.

---

## Sources

- UA / Teletronix LA-2A manual (LOC mirror): https://tile.loc.gov/storage-services/master/mbrs/recording_preservation/manuals/Teletronix%20Model%20LA-2A%20Leveling%20Amplifier.pdf
- UA LA-2A manual: https://media.uaudio.com/assetlibrary/l/a/la-2a_manual.pdf
- crochambeau, "continuing contemplation of Teletronix": http://crochambeau.blogspot.com/2011/10/continuing-contemplation-of-teletronix.html
- GroupDIY, "T4 optical attenuator for Teletronix LA-2A ... how the compressors works": https://groupdiy.com/threads/t4-optical-attenuator-for-teletronix-la-2a-and-how-the-compressors-works.80210/
- GroupDIY, "T4B Photocells": https://groupdiy.com/threads/t4b-photocells.64961/
- GroupDIY, "LA-2A 6AQ5W screen voltage and 12AX7 plate voltage": https://groupdiy.com/threads/la-2a-6aq5w-screen-voltage-and-12ax7-plate-voltage.87100/
- AudioScape, "Why do we make our own T4B Optical Cells?": https://www.audio-scape.com/news/t4b
- AudioTechnology, "On The Bench: Compressors Part 2 — The Side-Chain": http://www.audiotechnology.com.au/wp/index.php/on-the-bench-compressors-part-2-the-side-chain/
- KVR, "Help Understanding the LA-2A": https://www.kvraudio.com/forum/viewtopic.php?t=514970
- analogvibes, "Teletronix LA2A - DIY Tube Opto Compressor": https://analogvibes.com/know-how/la2a-tube-opto-compressor/
- Wikipedia, "LA-2A Leveling Amplifier": https://en.wikipedia.org/wiki/LA-2A_Leveling_Amplifier
