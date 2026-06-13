# Outboard Control Sweeps — LA-2A & Pultec (2026-06-12)

Control-sweep verification of the two working outboard examples on this
branch, using the `tests/audio_analysis.rs` measurement layer
(recompile-per-point, Goertzel on integer cycle counts, 48 kHz).

- Circuits: `examples/outboard/compressor/opto_leveler.pedal`,
  `examples/outboard/eq/passive_lc_eq.pedal`
- Tests: `tests/la2a_sweeps.rs` (2 green, 1 red-ignored),
  `tests/pultec_sweeps.rs` (6 green, 3 red-ignored)
- Every steady-state LA-2A point runs 8 s of program and measures the final
  second (T4B CdS release ~2.2 s; `opto_steady_gain_db` copied from
  `tests/outboard_acceptance.rs`).

---

## A. LA-2A (opto_leveler)

### A1. Gain pot sweep (0.2 amp, 1 kHz) — GAP, pot frozen

| Gain pos | steady gain (0.2 amp) | steady gain (0.002 amp) |
|---------:|----------------------:|------------------------:|
| 0.2 | +4.753 dB | +31.282 dB |
| 0.4 | +4.751 dB | +31.274 dB |
| 0.6 | +4.749 dB | +31.267 dB |
| 0.8 | +4.730 dB | +31.199 dB |

Not monotonic — flat within 0.09 dB and slightly **decreasing**. The
0.002-amp column rules out feedback-loop compensation (the sidechain is
nearly idle there and the pot is still dead). The Gain pot is a 3-terminal
divider into the 12AX7 grid; pots swallowed into an active device's group
go inert (same family as the F10 link-2 op-amp finding). Red test:
`la2a_sweeps.rs::gain_pot_raises_steady_gain` (`#[ignore]` with the
measured numbers).

### A2. GR vs level (Gain 0.6, 1 kHz) — GREEN, monotonic limiter

| in (dB) | gain (dB) | out (dB) |
|--------:|----------:|---------:|
| −40 | +24.279 | −15.721 |
| −35 | +20.888 | −14.112 |
| −30 | +17.261 | −12.739 |
| −25 | +13.473 | −11.527 |
| −20 |  +9.568 | −10.432 |
| −15 |  +5.573 |  −9.427 |
| −10 |  +1.507 |  −8.493 |
|  −5 |  −2.637 |  −7.637 |
|   0 |  −8.625 |  −8.625 |

- Gain falls **strictly monotonically** over the whole range (asserted,
  0.1 dB tolerance). There is no distinct knee — the feedback detector
  levels from −40 dB up (soft leveler, as the hardware).
- Total GR (−40 vs 0 dB input): **32.9 dB**.
- Effective ratio over the top region [−15, 0] dB (LSQ fit): **~15.3:1**
  (limiter-grade). In the top 5 dB the output actually *falls* ~1 dB as
  input rises 5 dB — over-unity limiting.

### A3. Peak Reduction control — infeasible in current DSL, documented

The real unit's sidechain-drive knob cannot be honestly expressed:

- `controls { ... }` entries bind **pot positions only**
  (`compiler/spqr_control.rs`; docs/dsl.md).
- The envelope binding resolves only a **direct** `EF1.out -> PC1.led` net
  (`compiler/bind.rs::build_envelope_jfet_bindings` scans `net.from ==
  EF.out` into a pin with a `modulation_sink()`); a pot inserted between
  `EF1.out` and `PC1.led` is not a modulation sink and would silently kill
  the sidechain.
- `envelope_follower(...)`'s `sensitivity_r` is a fixed constructor
  parameter, not control-bindable.

Candidate engine feature: **control-bindable EF sensitivity**. Documented
in the circuit header (note 5) — no dishonest pot was added.

### A4. Program-dependent release — measured FALSE, printed not asserted

| program before step-down | release (to ±1 dB) |
|--------------------------|-------------------:|
| heavy: 0.5 amp for 3.0 s | **1.494 s** |
| light: 0.15 amp for 0.5 s | **2.113 s** |

T4B memory predicts heavy > light; measured the opposite. The .pedal
collapses the T4B's dual time constants into one RC (header simplification
3), so no memory effect exists to measure. Test asserts only sanity bounds
(0.1–10 s) and prints the comparison.

Baseline (unchanged, from `outboard_acceptance`): GR 16.0 dB
(quiet +14.3 / loud −1.8 dB), attack 2.0 ms, release 1.78 s.

---

## B. Pultec (passive_lc_eq)

### Circuit edits made (kept)

1. **12AX7 makeup stage re-attempted — SUCCESS, kept.** AC-coupled
   common-cathode 12AX7 (100k plate, bypassed 1.5k cathode, B+ 250 V; the
   working pattern from `tests/passive_claiming.rs` / opto_leveler) after
   the network. With the F10 claiming fix the passives are no longer
   flattened and **all pot authority survives** (v1's four failed attempts
   are obsolete; header note 1 updated). Net chain gain ~+4.3 dB at 1 kHz.
2. **LF cut section added** (header section 0): LF_Cut pot (10k) in the
   series arm with a 47n HF bypass cap — RC shelf, corner ~339 Hz at full
   rotation. **Workaround required:** wiring the pot and cap as exact
   parallel edges between the same two nodes collapsed the *entire* network
   to silence (every probe < −157 dB); a 100R separation resistor
   (`R_lcsep`) in the cap branch restores normal compilation (new collapse
   trigger, header note 3).
3. Switched parts (F7) left as fixed values: switches now compile but are
   frozen at position 0, which would pin the EQ to the wrong taps
   (150n/27m = 106 Hz / 6.5 kHz); header note 2 updated.

### Full response (25 log points, dB; 0.1-amp probes)

| freq (Hz) | flat | LF boost=1 | LF cut=1 | HF boost=1 | HF atten=1 |
|----------:|-----:|-----:|-----:|-----:|-----:|
| 20 | 4.36 | 7.97 | 2.53 | 4.33 | 4.36 |
| 47 | 5.76 | 8.86 | 3.75 | 5.68 | 5.77 |
| 63 | 5.91 | 8.70 | 3.91 | 5.83 | 5.93 |
| 113 | 5.94 | 7.72 | 4.06 | 5.96 | 5.98 |
| 200 | 5.54 | 6.00 | 4.04 | 5.84 | 5.66 |
| 356 | 4.77 | 4.45 | 4.00 | 5.35 | 5.00 |
| 632 | 4.21 | 3.87 | 4.18 | 4.97 | 4.39 |
| 1125 | 4.28 | 4.12 | 4.49 | 5.12 | 3.91 |
| 2000 | 4.55 | 4.49 | 4.68 | 5.42 | 3.24 |
| 4743 | 4.74 | 4.73 | 4.77 | 5.62 | 2.41 |
| 8434 | 4.75 | 4.75 | 4.76 | 5.63 | 2.18 |
| 11247 | 2.96 | 2.96 | 2.97 | 3.83 | 0.34 |
| 14998 | −32.2 | −32.2 | −32.2 | −31.3 | −34.9 |
| 20000 | −108.6 | −108.6 | −108.6 | −107.9 | −111.3 |

(The ≥12 kHz collapse is an engine artifact present in the passive-only
version too — header note 7. Assertions stay ≤ 11 kHz.)

### Sweep results

| sweep | result | numbers |
|-------|--------|---------|
| LF boost monotonic shelf (63 & 100 Hz) | **GREEN** over positions {0, 0.5, 0.6, 0.75, 0.9, 1.0} | authority +2.80 dB @63 Hz, +2.03 dB @100 Hz; leak at 1 kHz −0.16 dB (< 2 dB) |
| LF boost full-range | **RED (#[ignore])** | pos 0.1 → +3.084 dB (2.8 dB *below* the 1 Ω floor), pos 0.5 ≡ pos 0.0 |
| HF boost peak | **GREEN** | max delta **+1.39 dB at 5044 Hz**; analytic f0 = 1/(2π√(0.047·22n)) = **4949 Hz** — well within an octave; boost monotonic over controls {0..0.5} |
| HF boost full-range | **RED (#[ignore])** | control 0.75 → +8.20 dB, control 1.0 → +6.19 dB ≡ control 0.5 |
| HF atten cut at 10 kHz | **GREEN** over controls {0..0.75} | monotonic, 6.18 dB of cut; full-throw (control 1.0) lands aliased at −2.6 dB ≡ control 0.5 |
| HF atten full-range | **RED (#[ignore])** | control 1.0 ≡ control 0.5 instead of deepest cut |
| THE TRICK (LF boost 0.7 + cut 0.7) | **GREEN** | sub-100 Hz lift vs flat +0.35..+0.83 dB (asserted > +0.2); mean dip vs boost-only over 200–500 Hz **−0.44 dB** (asserted < −0.2); deepest dip vs flat **−0.54 dB at ~356 Hz** (asserted < −0.3) |
| insertion gain sanity | **GREEN** | see table below, |Δ| = 3.5 dB < 6 dB |

### Analytic vs measured

| quantity | analytic (netlist math) | measured | Δ |
|----------|------------------------:|---------:|---:|
| HF boost resonance f0 | 4949 Hz | peak at 5044 Hz | +0.03 oct |
| HF boost peak height (BW=0) | ~+13.6 dB (R2 bypass divider) | +1.39 dB | **−12.2 dB — engine gap** |
| passive pad @1 kHz | −27.7 dB | −28.9 dB (v1 measurement) | 1.2 dB |
| 12AX7 stage gain | +35.5 dB (μ100, rp 62.5k, 100k∥1M) | — | — |
| chain gain @1 kHz | +7.75 dB | +4.23 dB | 3.5 dB ✓ (<6) |
| LF boost corner | ~59 Hz (1/(2π·10k·270n)) | shelf max at ~47–63 Hz | ✓ |
| LF cut corner (full) | ~339 Hz (1/(2π·10k·47n)) | transition 150–630 Hz | ✓ |

---

## Gaps found (new this pass)

1. **Pot position aliasing via `set_control`** (pultec header note 8): in
   compiled passive trees, positions < 0.5 are non-physical — position 0.5
   measures *identical* to position 0.0, and positions ~0.1–0.25 land
   outside the physically reachable range (LF Boost 0.1 → 2.8 dB below the
   1 Ω floor; LF Cut 0.1 → 2.1 dB above flat). Linear and audio tapers
   both affected. Upper half of travel behaves physically.
2. **Compiled-in control defaults are inert** for those pots — response
   always matches position 0 regardless of `controls { ... = X }`.
3. **Exact-parallel-edge collapse**: a pot and a cap wired between the same
   two nodes silence the whole network (< −157 dB everywhere); a 100R
   separation resistor is the workaround (pultec header note 3).
4. **LC branch reactance mostly lost**: the HF boost's series-resonant
   branch should peak ~+13.6 dB at minimum damping; measured a broad
   +1.4 dB hump (peak *location* is correct).
5. **Steep >12 kHz rolloff artifact** regardless of controls (−32 dB at
   15 kHz, < −100 dB at 20 kHz), passive-only version included.
6. **opto_leveler Gain pot frozen** (3-terminal divider into triode grid) —
   see A1.
7. **Peak Reduction not expressible** — candidate engine feature:
   control-bindable envelope-follower sensitivity (A3).
8. **No T4B release memory** — heavy-GR release measures *faster* than
   light-GR release (1.49 s vs 2.11 s), opposite the hardware (A4);
   consequence of the single-RC release simplification.

## Realtime performance note

From the branch's earlier benchmark pass (release build, single core,
48 kHz): "opto_leveler 7.5x realtime, fet_leveler 11.4x, dyna_comp 63x,
tube_screamer 46x". The 8-s-per-point T4B settle is what makes the LA-2A
sweeps the slowest tests here (~12 s for the GR curve at 7.5x realtime);
the Pultec suite runs ~20 s total.

## Verification

- `cargo test -p pedalkernel --release --no-default-features --test
  la2a_sweeps` → 2 passed, 1 ignored (red, measured numbers in reason).
- `... --test pultec_sweeps` → 6 passed, 3 ignored (red).
- `... --test outboard_acceptance` → 8 passed, 1 ignored (unchanged);
  `outboard_examples` → 8 passed.
- rustfmt + clippy clean on the two new test files.
