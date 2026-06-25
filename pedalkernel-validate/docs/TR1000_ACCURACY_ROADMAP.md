# TR-1000 Voice Accuracy Roadmap

Living roadmap tracking SPICE validation coverage and WDF accuracy gaps for all
TR-1000 drum voices. Updated as batches land. Worst-score-first drives fix order.

**Batch status:**
- B1 (lq6.4 + lq6.6): 808 kick + snare + tom + mid_tom + hi_tom — SPICE decks done, all in drums suite, all RED (known_gap)
- B2 (lq6.7): Remaining 808 voices (clap, claves, closed_hat, open_hat, cowbell, maracas, rimshot) — DONE. All 7 SPICE decks + drums suite cases added. 1/7 B2 voices passes gate (closed\_hat RMS +8.3dB < 20dB threshold).
- B3 (lq6.8): 909 voices (kick, snare, clap, hi_tom, mid_tom, lo_tom, rim) + 606 snare — DONE. All 8 SPICE decks + drums suite cases added. 0/8 B3 voices passes gate (all known\_gap: lq6.1 resonator bug dominates).

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
| 909 kick | done (B3) | yes | +47.1 | +72.7 | known\_gap | Bridged-T body approx (f0=48.2 Hz, Q=5.71; R=330k, C=10n, R\_fb=1200k). SPICE: state-space G-source for .pedal bridged-T body only. GAP 1: Authentic 909 kick uses 2SC1583 dual-NPN transistor multivibrator (BJT oscillator, convergence-hard — too stiff for SPICE harness transient solver); behavioral body substituted. GAP 2: PitchEnvelope sweep (190→45 Hz over 80ms, amount=3.2) not in SPICE. GAP 3: VoiceNoise (4kHz, decay=8ms) + VoiceClick (3ms) DSP layers not modeled. Highest peak error of B3 (+72.7dB) — three stacked gaps + lq6.1. | lq6.1 WDF resonator bug. After fix: BJT oscillator gap + PitchEnvelope gap will remain as separate modeling limitations. |
| 909 snare | done (B3) | yes | +60.9 | +44.8 | known\_gap | Dual bridged-T (lo f0=338.6 Hz Q=5.0 + hi f0=451.4 Hz Q=6.0; R\_lo=100k, R\_hi=75k, C=4.7n). Higher-pitched than 808 snare (~154+339 Hz) — characteristic 909 brightness. SPICE: two state-space resonators summed. GAP: VoiceNoise (6kHz, Q=2.5, decay=60ms, amount=0.7) not modeled. | lq6.1 WDF resonator bug + noise path gap. |
| 909 clap | done (B3) | yes | +26.9 | +41.2 | known\_gap | Single bridged-T body resonator (f0=1538.5 Hz, Q=1.43; R=22k, C=4.7n, R\_fb=220k). Identical resonator to 808 clap — distinction is 4-burst (909) vs 3-burst (808) DSP envelope pattern. SPICE: state-space G-source body only. GAP 1: ClapEnvelope909 4-burst DSP envelope not in WDF path. GAP 2: noise source not modeled. | lq6.1 WDF resonator bug + two envelope/noise gaps (same pattern as 808 clap). |
| 909 hi tom | done (B3) | yes | +39.3 | +61.1 | known\_gap | Single bridged-T (f0=188.4 Hz, Q=4.86; R=180k, C=4.7n, R\_fb=680k). Authentic TR-909 HT uses UJT relaxation oscillator (~140-160 Hz). WDF loading shifts resonance toward 190-210 Hz. SPICE: state-space G-source. GAP: PitchEnvelope DSP sweep (decay=30ms, amount=0.5) not in SPICE. | lq6.1 WDF resonator bug. |
| 909 mid tom | done (B3) | yes | +40.7 | +64.5 | known\_gap | Single bridged-T (f0=125.3 Hz, Q=5.26; R=270k, C=4.7n, R\_fb=1000k). Authentic TR-909 MT uses UJT relaxation oscillator (~110-120 Hz). SPICE: state-space G-source. GAP: PitchEnvelope DSP sweep (decay=30ms, amount=0.5) not in SPICE. | lq6.1 WDF resonator bug. |
| 909 lo tom | done (B3) | yes | +44.4 | +68.5 | known\_gap | Single bridged-T (f0=79.6 Hz, Q=5.0; R=200k, C=10n, R\_fb=750k). Authentic TR-909 LT uses UJT relaxation oscillator (~65 Hz). SPICE: state-space G-source. GAP: PitchEnvelope DSP sweep (decay=30ms, amount=0.5) not in SPICE. | lq6.1 WDF resonator bug. |
| 909 rim | done (B3) | yes | +32.3 | +54.0 | known\_gap | Single bridged-T (f0=412.3 Hz, Q=5.56 unloaded; R=82k, C=4.7n, R\_fb=300k). SPICE uses unloaded Q (Decay=1.0); WDF applies pot damping internally. Decay=0.3 default gives short click (~1-2ms ring). Same pitch range as 808 rimshot — character differentiation by shorter decay, not pitch. | lq6.1 WDF resonator bug + WDF bias-loading shift (same pattern as 808 rimshot). |

## 606 Voices

| Voice | SPICE deck | In drums suite | RMS error (dB) | Peak error (dB) | Profile | Topology / modeling approach | Gap to close |
|---|---|---|---|---|---|---|---|
| 606 snare | done (B3) | yes | +62.4 | +47.7 | known\_gap | Dual bridged-T (lo f0=282.3 Hz Q=4.27 + hi f0=338.6 Hz Q=5.62; R\_lo=120k, R\_hi=100k, C=4.7n). Note: Hi-body R\_fb raised 330k→365k to fix prior imbalance (hi was 10× louder). SPICE: two state-space resonators summed. GAP: noise burst (VoiceNoise DSP layer) not modeled. | lq6.1 WDF resonator bug + noise path gap. |

---

## Fix Priority (worst score first, B1+B2+B3 combined)

1. **lq6.1** (root cause for all resonator voices): WDF bridged-T resonator compiles with f0 at
   Nyquist. All 20 voices are affected. Score ordering worst-first (B3 voices in **bold**):
   - snare +65.1 RMS
   - **606\_snare +62.4 RMS** (dual resonator; same Nyquist error as 808 snare)
   - cowbell +60.1 RMS (dual resonator compounds the Nyquist error)
   - **909\_snare +60.9 RMS** (dual resonator)
   - **909\_kick +47.1 RMS** (triple gap: BJT oscillator + PitchEnvelope + noise)
   - **909\_lo\_tom +44.4 RMS**, **909\_mid\_tom +40.7 RMS**, **909\_hi\_tom +39.3 RMS**
   - mid\_tom +38.2, hi\_tom/tom +37.7, kick +35.2
   - **909\_rim +32.3**, rimshot +32.4, clap +26.9, **909\_clap +26.9**, claves +23.2
   - open\_hat +17.2, maracas +10.1, closed\_hat +8.3

2. **808/909 rimshot WDF loading shift**: After lq6.1 fix, the rimshot resonator f0 will shift
   from Nyquist to the WDF-computed frequency (~1477 Hz with bias-divider loading, vs theoretical
   1026 Hz). The 909 rim shares the same bias-loading effect. Tracked in
   ENGINE\_BUG\_BRIDGED\_T\_BIAS\_LOADING.md.

3. **Noise/envelope gaps** (808 clap, 808 maracas, 808 snare, 909 clap, 909 snare, 606 snare):
   After lq6.1 fix, these voices will still diverge from authentic hardware due to missing noise
   sources and multi-burst ClapEnvelope DSP blocks. Separate work items for each.

4. **909 kick BJT oscillator + PitchEnvelope gaps**: After lq6.1, the 909 kick will still fail
   because (a) the .pedal uses a bridged-T body approx instead of the authentic 2SC1583 BJT
   multivibrator; (b) the PitchEnvelope (190→45 Hz sweep) is a DSP-only layer not in the WDF
   comparison path. These are .pedal modeling limitations, not engine bugs.

5. **Six-oscillator metallic source gap** (808 closed\_hat, open\_hat, cowbell): The .pedal files use
   single/dual resonators as approximations. The authentic metallic source (6 square-wave osc)
   is not modeled at the WDF level — modeling limitation of the .pedal design, not an engine bug.

6. **808\_tom / 808\_hi\_tom component identity**: Both use identical R/C/Q values. If different
   tuning is intended, pedal files need updating before the SPICE decks diverge.

---

*Updated: 2026-06-25 (B3 complete — all 20 TR-1000 voices measured). Scores from report.json (drums suite, bd-pedalkernel-lq6.4 branch).*
