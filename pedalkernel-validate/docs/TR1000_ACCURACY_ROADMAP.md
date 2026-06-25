# TR-1000 Voice Accuracy Roadmap

Living roadmap tracking SPICE validation coverage and WDF accuracy gaps for all
TR-1000 drum voices. Updated as batches land. Worst-score-first drives fix order.

**Batch status:**
- B1 (lq6.4 + lq6.6): 808 kick + snare + tom + mid_tom + hi_tom — SPICE decks done, all in drums suite, all RED (known_gap)
- B2 (lq6.7): Remaining 808 voices (clap, claves, closed_hat, open_hat, cowbell, maracas, rimshot) — DONE. All 7 SPICE decks + drums suite cases added. 1/7 B2 voices passes gate (closed\_hat RMS +8.3dB < 20dB threshold).
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
| 808 clap | done (B2) | yes | +26.9 | +41.2 | known\_gap | Single bridged-T body resonator (f0=1539 Hz, Q=1.43; R=22k, C=4.7n, R\_fb=220k). SPICE: state-space G-source. GAP 1: multi-burst comb envelope (ClapEnvelope DSP block, not in WDF path). GAP 2: noise source not modeled. Three stacked limitations. | lq6.1 WDF resonator bug + two noise/envelope gaps. Fix order: lq6.1 first (resonator); noise+envelope gaps are separate work items. |
| 808 claves | done (B2) | yes | +23.2 | +38.0 | known\_gap | Single bridged-T (f0=2257.5 Hz, Q=23.5; R=150k, C=470p, R\_fb=470k). SPICE: state-space G-source. Short percussive click. | lq6.1 WDF resonator bug. Expect improvement after fix. |
| 808 closed hat | done (B2) | yes | +8.3 | +18.1 | known\_gap | 7074 Hz resonator approx (Q\_eff≈5). R\_fb=47k < r\_crit=450k → sub-critical, highly damped. **PASSES 20dB threshold** despite WDF lq6.1 bug — error is lower because both WDF and SPICE produce a very short transient at high freq. GAP: 6-osc metallic source not modeled (single-resonator .pedal approximation only). | lq6.1 fix may improve or worsen the gate — re-measure after fix. 6-osc metallic source gap is a separate modeling limitation of the .pedal design (not an engine bug). |
| 808 open hat | done (B2) | yes | +17.2 | +28.3 | known\_gap | 7074 Hz resonator (Q=23.5; R=150k, C=150p, R\_fb=470k > r\_crit=450k → above critical). SPICE: state-space G-source. Same f0/Q as open hat (R/C values identical to closed hat). GAP: 6-osc metallic source not modeled. | lq6.1 WDF resonator bug. Authentic open-hat decay (~300ms) not achievable with single-resonator .pedal topology (modeling limitation, not engine bug). |
| 808 cowbell | done (B2) | yes | +60.1 | +39.1 | known\_gap | Dual bridged-T: lo (f0=720.5 Hz, Q=2.09; R=47k, C=4.7n) + hi (f0=1026.1 Hz, Q=1.98; R=33k, C=4.7n). SPICE: two state-space resonators summed. GAP: authentic TR-808 cowbell uses square-wave oscillators (~560/845 Hz) — sinusoidal resonator approximation only. WDF loading shifts actual frequencies below textbook. | lq6.1 WDF resonator bug (highest RMS score of B2 at +60.1dB — dual resonator makes the Nyquist error compound). Square-wave harmonic gap is a separate .pedal modeling limitation. |
| 808 maracas | done (B2) | yes | +10.1 | +26.8 | known\_gap | Single bridged-T click resonator (f0=4822.9 Hz, Q=34; R=220k, C=150p, R\_fb=680k). SPICE: state-space G-source. GAP: real maracas character is DSP noise (VoiceNoise at ~6 kHz bandpass, outside WDF model). Tonal body models only the attack click. | lq6.1 WDF resonator bug. Noise path gap is a separate DSP layer (not engine bug). |
| 808 rimshot | done (B2) | yes | +32.4 | +47.2 | known\_gap | Single bridged-T (f0=1026.1 Hz, Q=2.78; R=47k, C=3.3n, R\_fb=220k), τ≈0.86ms. SPICE: state-space G-source. Very short click. WDF loading raises measured f0 to ~1477 Hz (WDF bias-loading effect documented in ENGINE\_BUG\_BRIDGED\_T\_BIAS\_LOADING.md). | lq6.1 WDF resonator bug + WDF bias-loading frequency shift (separate from lq6.1 — the loading shifts f0 even after lq6.1 is fixed). |

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

## Fix Priority (worst score first, B1+B2 combined)

1. **lq6.1** (root cause for all resonator voices): WDF bridged-T resonator compiles with f0 at
   Nyquist. All 12 voices are affected. Score ordering worst-first (B2 voices in bold):
   - snare +65.1 RMS
   - **cowbell +60.1 RMS** (dual resonator compounds the Nyquist error)
   - mid\_tom +38.2, hi\_tom/tom +37.7, kick +35.2
   - **rimshot +32.4**, **clap +26.9**, **claves +23.2**
   - **open\_hat +17.2**, **maracas +10.1**, **closed\_hat +8.3**

2. **808 rimshot WDF loading shift**: After lq6.1 fix, the rimshot resonator f0 will shift
   from Nyquist to the WDF-computed frequency (~1477 Hz with bias-divider loading, vs theoretical
   1026 Hz). This is a separate engine gap (bridged-T bias-loading). Tracked in
   ENGINE\_BUG\_BRIDGED\_T\_BIAS\_LOADING.md.

3. **Noise/envelope gaps** (clap, maracas, snare): After lq6.1 fix, these voices will still
   diverge from authentic hardware due to missing noise sources and the clap's multi-burst
   ClapEnvelope DSP block. Separate work items for each.

4. **Six-oscillator metallic source gap** (closed\_hat, open\_hat, cowbell): The .pedal files use
   single/dual resonators as approximations. The authentic metallic source (6 square-wave osc)
   is not modeled at the WDF level — this is a modeling limitation of the .pedal design, not
   an engine bug. Separate work items if authentic metallic character is needed.

5. **Snare noise gap** (also applies to 808\_snare B1): Tonal body will pass after lq6.1,
   but noise burst component remains unmodeled.

6. **808\_tom / 808\_hi\_tom component identity**: Both use identical R/C/Q values. If different
   tuning is intended, pedal files need updating before the SPICE decks diverge.

---

*Updated: 2026-06-25 (B2 complete). Scores from report.json (drums suite, bd-pedalkernel-lq6.4 branch).*
