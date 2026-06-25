# TR-1000 Voice Accuracy Roadmap

Living roadmap tracking SPICE validation coverage and WDF accuracy gaps for all
TR-1000 drum voices. Updated as batches land. Worst-score-first drives fix order.

**Batch status:**
- B1 (lq6.4 + lq6.6): 808 kick + snare + tom + mid_tom + hi_tom — SPICE decks done, all in drums suite, all RED (known_gap)
- B2 (lq6.7): Remaining 808 voices (clap, claves, closed_hat, open_hat, cowbell, maracas, rimshot)
- B3 (lq6.8): 909 voices (kick, snare, clap, hi_tom, mid_tom, lo_tom, rim) + 606 snare

**How to read scores:** RMS and Peak errors are in dB relative to the SPICE
behavioral golden.  Positive values mean the WDF output is louder or more
distorted than reference — large positive values indicate a broken voice
(e.g. resonator at Nyquist instead of target f0, per lq6.1).

---

## 808 Voices

| Voice | SPICE deck | In drums suite | RMS error (dB) | Peak error (dB) | Profile | Topology / modeling approach | Gap to close |
|---|---|---|---|---|---|---|---|
| 808 kick | done | yes | +35.2 | +63.3 | known\_gap | Bridged-T Sallen-Key resonator. SPICE: state-space G-source (f0=129.4 Hz, Q≈30). | lq6.1: WDF f0 appears at Nyquist instead of 129 Hz. Root cause: bridged-T RC not classified as resonator stage. |
| 808 snare | done | yes | +65.1 | +50.1 | known\_gap | Dual bridged-T: lo (f0=153.8 Hz, Q=2.5) + hi (f0=338.6 Hz, Q=5.0) summed. SPICE: two state-space resonators; noise burst NOT modeled (documented gap). | Same lq6.1 WDF resonator bug. Noise path is a second gap (no noise source in WDF model). |
| 808 tom | done | yes | +37.7 | +62.3 | known\_gap | Single bridged-T (f0=153.8 Hz, Q=8.33). SPICE: state-space G-source. Note: 808\_tom.pedal is internally labelled "808 Mid Tom" — see naming note in 808\_tom.spice. | lq6.1 WDF resonator bug. |
| 808 mid tom | done | yes | +38.2 | +64.0 | known\_gap | Single bridged-T (f0=125.3 Hz, Q=9.1). SPICE: state-space G-source. | lq6.1 WDF resonator bug. |
| 808 hi tom | done | yes | +37.7 | +62.3 | known\_gap | Single bridged-T (f0=153.8 Hz, Q=8.33). Same component values as 808\_tom.pedal (R=220k, C=4.7n, R\_fb=750k) — identical SPICE behavior. See 808\_hi\_tom.spice naming note. | lq6.1 WDF resonator bug. Same values as 808\_tom: two separate voices with identical component tuning. |
| 808 clap | todo (B2) | no | — | — | pending | Multiple noise bursts through short reverb decay (not a resonator). Likely needs noise-burst + IIR decay SPICE model. | Noise source not in WDF voice model at all. |
| 808 claves | todo (B2) | no | — | — | pending | High-Q short-decay resonator (~2.5 kHz, Q≈5). Single bridged-T. | Same lq6.1 WDF resonator bug (different f0 range). |
| 808 closed hat | todo (B2) | no | — | — | pending | Six-oscillator metallic tone (overlapping square waves + bandpass). Complex topology — full opamp simulation may be needed or state-space multi-resonator model. | WDF metallic-hat model not yet implemented. |
| 808 open hat | todo (B2) | no | — | — | pending | Same six-oscillator source as closed hat + longer decay. | Same as closed hat. |
| 808 cowbell | todo (B2) | no | — | — | pending | Two square-wave oscillators + bandpass. Similar multi-resonator approach to hat. | WDF cowbell model not yet implemented. |
| 808 maracas | todo (B2) | no | — | — | pending | Noise burst + bandpass. Similar to clap topology. | Noise source gap. |
| 808 rimshot | todo (B2) | no | — | — | pending | Short transient click + brief resonance. Likely impulse + single low-Q resonator. | To be measured after .pedal read. |

## 909 Voices

| Voice | SPICE deck | In drums suite | RMS error (dB) | Peak error (dB) | Profile | Topology / modeling approach | Gap to close |
|---|---|---|---|---|---|---|---|
| 909 kick | todo (B3) | no | — | — | pending | BJT-based bridged-T relaxation oscillator with pitch sweep (experimental UJT variant also exists). Different topology from 808 kick — BJT NPN pairs. | BJT two-port solver implemented but 909 kick WDF voice not validated. |
| 909 snare | todo (B3) | no | — | — | pending | Bridged-T resonator + noise. Similar dual-tone approach to 808 snare. | Noise path gap + WDF resonator bug (f0 may differ). |
| 909 clap | todo (B3) | no | — | — | pending | Multiple overlapping noise bursts (reverb effect). | Noise source gap. |
| 909 hi tom | todo (B3) | no | — | — | pending | Bridged-T resonator (higher f0 than 808 toms). | WDF resonator bug. |
| 909 mid tom | todo (B3) | no | — | — | pending | Bridged-T resonator (mid f0). | WDF resonator bug. |
| 909 lo tom | todo (B3) | no | — | — | pending | Bridged-T resonator (low f0, longer decay). | WDF resonator bug. |
| 909 rim | todo (B3) | no | — | — | pending | Short click + brief resonance. | To be measured. |

## 606 Voices

| Voice | SPICE deck | In drums suite | RMS error (dB) | Peak error (dB) | Profile | Topology / modeling approach | Gap to close |
|---|---|---|---|---|---|---|---|
| 606 snare | todo (B3) | no | — | — | pending | Bridged-T resonator + noise (simpler than 808/909 snare). | WDF resonator bug + noise gap. |

---

## Fix Priority (worst score first, B1 batch)

1. **lq6.1** (root cause for all resonator voices): WDF bridged-T resonator compiles with f0 at
   Nyquist. All 5 B1 voices are blocked on this single fix. Once resolved, all five should move
   from RMS +35–65 dB to near 0 dB error (the SPICE state-space goldens are behaviorally
   correct). Score ordering: snare (+65.1 RMS) > mid\_tom (+38.2) > hi\_tom/tom (+37.7) > kick (+35.2).

2. **Snare noise gap**: After lq6.1 fix, the tonal body will be validated but the noise burst
   component remains unmodeled. A separate SPICE deck with a noise source + bandpass would be
   needed for full snare character. Low priority until tonal body passes.

3. **808\_tom / 808\_hi\_tom component identity**: Both use identical R/C/Q values. If the intent
   is different tuning for hi vs lo tom, the pedal files need updating before the SPICE decks
   diverge. Track as a design question in B2.

---

*Generated: 2026-06-25. Scores from report.json (drums suite, bd-pedalkernel-lq6.4 + lq6.6 branch).*
