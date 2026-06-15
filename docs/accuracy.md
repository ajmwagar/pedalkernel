---
title: "Accuracy Dashboard"
description: "Honest, mid-campaign snapshot of WDF engine output vs ngspice golden references — every circuit, real dB errors, known gaps included."
section: "Internals"
weight: 101
---

# Accuracy Dashboard

This page reports how closely the PedalKernel WDF engine matches its **ngspice golden references**, circuit by circuit. It is generated from committed golden `.npy` files — the engine's own WDF output compared against SPICE ground truth — by `tools/dashboard/generate_dashboard.py`. See [Validation](/book/validation/) for how the goldens are produced.

> **This is an honest, mid-campaign snapshot.** Failures are shown, not hidden. The numbers below include the known accuracy gaps we are still working through (see [Known Gaps](#known-gaps)).

## Summary

- **Run timestamp:** `2026-06-15T00:00:00Z`
- **Git commit:** `0028e6cee`
- **Sample rate:** 96000 Hz x 4 oversampling
- **Aggregate:** **91/100 signals pass** (91.0%), 9 fail, 0 WDF pending

Per-suite breakdown:

| Suite | Pass / Total |
|-------|--------------|
| active | 14 / 14 |
| canonical | 23 / 23 |
| compressor | 0 / 2 |
| eq | 10 / 10 |
| extraction | 8 / 11 |
| linear | 7 / 7 |
| nonlinear | 10 / 10 |
| opamp | 6 / 6 |
| pedals | 4 / 8 |
| reactive | 6 / 6 |
| stress | 1 / 1 |
| tape | 2 / 2 |

## Drift History

Append-only per-run pass-rate trend (each dashboard run appends one record):

| Run | Timestamp | Commit | Pass rate | Passed / Total |
|-----|-----------|--------|-----------|----------------|
| 1 | `2026-06-15T00:00:00Z` | `0028e6cee` | 91.0% | 91 / 100 |

_This is the first recorded run — the baseline. Subsequent runs will reveal drift (regressions or improvements) against it._

## Accuracy Matrix

Every circuit x signal, colored by error magnitude (green = close to ngspice, red = far). Columns are normalized RMS error, peak error, and spectral error (all in dB; lower is better).

![Accuracy matrix: all circuits by RMS / peak / spectral error vs ngspice](/accuracy/accuracy-matrix.png)

## Known Gaps

The current failing circuits and the reasons we already understand:

| Circuit | Symptom | Notes |
|---------|---------|-------|
| `extraction/sallen_key_highpass` | ~ +13 dB RMS/peak | High-pass filter gain offset vs SPICE |
| `extraction/mfb_highpass_controlled` | spectral mismatch | MFB high-pass topology gap |
| `compressor/fet_leveler_*` | high spectral / THD error | FET leveler gain-cell modeling gap |
| `pedals/bigmuff_stage` | large RMS/peak error | Big Muff clipping stage operating point |
| `pedals/fuzz_core` | RMS/THD error | Fuzz operating-point / bias gap |

These are tracked openly rather than masked — the dashboard exists to make drift visible. The Tube Screamer hot-signal +5.4 dB and Klon gain deltas noted elsewhere in the campaign are within the per-circuit tables below where they apply.

## Per-Circuit Detail

Each entry shows the time-domain overlay (ngspice vs WDF + difference) and the FFT magnitude overlay, annotated with the dB-error metrics. Metrics legend: **RMS** = normalized RMS error, **Peak** = peak error, **Spec** = spectral error, **THD** = THD error (dB; `—` when not applicable).

### active

| Circuit / signal | Pass | RMS (dB) | Peak (dB) | Spec (dB) | THD (dB) |
|------------------|------|----------|-----------|-----------|----------|
| fuzz_face_pnp / clean | PASS | 1.1 | 1.4 | 22.4 | 34.8 |
| fuzz_face_pnp / saturated | PASS | 1.1 | 1.3 | 76.7 | 63.7 |
| jfet_source_follower / sine | PASS | 0.0 | 0.0 | 173.4 | 86.6 |
| nmos_common_source / sine | PASS | -0.3 | -0.2 | 208.3 | 137.4 |
| npn_common_emitter / clean | PASS | -0.1 | -0.1 | 216.4 | 133.5 |
| npn_common_emitter / driven | PASS | -0.5 | -0.3 | 208.8 | 58.4 |
| optical_attenuator / sine | PASS | -48.0 | -16.7 | 167.5 | 1.2 |
| ota_ca3080 / sine | PASS | 0.0 | 0.0 | 600.4 | 45.4 |
| phase_allpass_stage / sine | PASS | 0.0 | 0.0 | 234.1 | 108.2 |
| phase_allpass_stage / sweep | PASS | 0.0 | 0.0 | 278.4 | — |
| pmos_source_follower / sine | PASS | 0.0 | 0.7 | 138.4 | 149.3 |
| pnp_common_emitter / clean | PASS | 0.0 | 0.1 | 98.0 | 96.2 |
| pnp_common_emitter / driven | PASS | 0.0 | 0.4 | 82.5 | 111.9 |
| push_pull_6l6 / sine | PASS | -0.0 | -0.0 | 76.8 | 36.2 |

![active/fuzz_face_pnp/clean — RMS 1.1 dB, peak 1.4 dB, spectral 22.4 dB](/accuracy/circuits/active/fuzz_face_pnp/clean.png)

*active/fuzz_face_pnp/clean — RMS 1.1 dB, peak 1.4 dB, spectral 22.4 dB*

![active/fuzz_face_pnp/saturated — RMS 1.1 dB, peak 1.3 dB, spectral 76.7 dB](/accuracy/circuits/active/fuzz_face_pnp/saturated.png)

*active/fuzz_face_pnp/saturated — RMS 1.1 dB, peak 1.3 dB, spectral 76.7 dB*

![active/jfet_source_follower/sine — RMS 0.0 dB, peak 0.0 dB, spectral 173.4 dB](/accuracy/circuits/active/jfet_source_follower/sine.png)

*active/jfet_source_follower/sine — RMS 0.0 dB, peak 0.0 dB, spectral 173.4 dB*

![active/nmos_common_source/sine — RMS -0.3 dB, peak -0.2 dB, spectral 208.3 dB](/accuracy/circuits/active/nmos_common_source/sine.png)

*active/nmos_common_source/sine — RMS -0.3 dB, peak -0.2 dB, spectral 208.3 dB*

![active/npn_common_emitter/clean — RMS -0.1 dB, peak -0.1 dB, spectral 216.4 dB](/accuracy/circuits/active/npn_common_emitter/clean.png)

*active/npn_common_emitter/clean — RMS -0.1 dB, peak -0.1 dB, spectral 216.4 dB*

![active/npn_common_emitter/driven — RMS -0.5 dB, peak -0.3 dB, spectral 208.8 dB](/accuracy/circuits/active/npn_common_emitter/driven.png)

*active/npn_common_emitter/driven — RMS -0.5 dB, peak -0.3 dB, spectral 208.8 dB*

![active/optical_attenuator/sine — RMS -48.0 dB, peak -16.7 dB, spectral 167.5 dB](/accuracy/circuits/active/optical_attenuator/sine.png)

*active/optical_attenuator/sine — RMS -48.0 dB, peak -16.7 dB, spectral 167.5 dB*

![active/ota_ca3080/sine — RMS 0.0 dB, peak 0.0 dB, spectral 600.4 dB](/accuracy/circuits/active/ota_ca3080/sine.png)

*active/ota_ca3080/sine — RMS 0.0 dB, peak 0.0 dB, spectral 600.4 dB*

![active/phase_allpass_stage/sine — RMS 0.0 dB, peak 0.0 dB, spectral 234.1 dB](/accuracy/circuits/active/phase_allpass_stage/sine.png)

*active/phase_allpass_stage/sine — RMS 0.0 dB, peak 0.0 dB, spectral 234.1 dB*

![active/phase_allpass_stage/sweep — RMS 0.0 dB, peak 0.0 dB, spectral 278.4 dB](/accuracy/circuits/active/phase_allpass_stage/sweep.png)

*active/phase_allpass_stage/sweep — RMS 0.0 dB, peak 0.0 dB, spectral 278.4 dB*

![active/pmos_source_follower/sine — RMS 0.0 dB, peak 0.7 dB, spectral 138.4 dB](/accuracy/circuits/active/pmos_source_follower/sine.png)

*active/pmos_source_follower/sine — RMS 0.0 dB, peak 0.7 dB, spectral 138.4 dB*

![active/pnp_common_emitter/clean — RMS 0.0 dB, peak 0.1 dB, spectral 98.0 dB](/accuracy/circuits/active/pnp_common_emitter/clean.png)

*active/pnp_common_emitter/clean — RMS 0.0 dB, peak 0.1 dB, spectral 98.0 dB*

![active/pnp_common_emitter/driven — RMS 0.0 dB, peak 0.4 dB, spectral 82.5 dB](/accuracy/circuits/active/pnp_common_emitter/driven.png)

*active/pnp_common_emitter/driven — RMS 0.0 dB, peak 0.4 dB, spectral 82.5 dB*

![active/push_pull_6l6/sine — RMS -0.0 dB, peak -0.0 dB, spectral 76.8 dB](/accuracy/circuits/active/push_pull_6l6/sine.png)

*active/push_pull_6l6/sine — RMS -0.0 dB, peak -0.0 dB, spectral 76.8 dB*

### canonical

| Circuit / signal | Pass | RMS (dB) | Peak (dB) | Spec (dB) | THD (dB) |
|------------------|------|----------|-----------|-----------|----------|
| bjt_diff_pair / linear | PASS | -0.0 | -0.0 | 96.6 | 114.2 |
| bjt_diff_pair / soft_clipping | PASS | -0.0 | -0.0 | 134.3 | 91.5 |
| bjt_diff_pair / sweep | PASS | 0.1 | -0.1 | 184.8 | — |
| opamp_diode_clipper / linear | PASS | 0.0 | 0.8 | 215.4 | 7.8 |
| opamp_diode_clipper / clipping | PASS | 0.1 | 1.2 | 208.6 | 22.2 |
| opamp_diode_clipper / sweep | PASS | 0.1 | 1.2 | 119.3 | — |
| rc_highpass / impulse | PASS | -247.2 | -247.6 | 0.0 | — |
| rc_highpass / sine | PASS | -283.5 | -282.6 | 0.0 | 0.0 |
| rc_highpass / sweep | PASS | -279.7 | -275.7 | 0.0 | — |
| rc_lowpass / impulse | PASS | -200.0 | -200.0 | 0.0 | — |
| rc_lowpass / sine | PASS | -307.7 | -297.6 | 0.0 | 0.0 |
| rc_lowpass / sweep | PASS | -308.3 | -298.3 | 0.0 | — |
| series_rlc / impulse | PASS | -205.6 | -206.0 | 0.0 | — |
| series_rlc / sine | PASS | -287.7 | -285.0 | 0.0 | 0.0 |
| series_rlc / sweep | PASS | -230.7 | -225.9 | 0.0 | — |
| single_diode / mild_clipping | PASS | -25.6 | -26.9 | 218.0 | 0.1 |
| single_diode / hard_clipping | PASS | -40.1 | -40.7 | 172.5 | 0.0 |
| single_diode / sweep | PASS | -5.2 | 0.2 | 69.3 | — |
| transformer_load / sine | PASS | -90.7 | -90.1 | 0.0 | 56.1 |
| transformer_load / sweep | PASS | -60.3 | -49.8 | 0.1 | — |
| twin_t_notch / impulse | PASS | -236.0 | -236.2 | 0.0 | — |
| twin_t_notch / sine | PASS | -242.4 | -237.0 | 0.0 | 0.0 |
| twin_t_notch / sweep | PASS | -269.1 | -262.6 | 0.0 | — |

![canonical/bjt_diff_pair/linear — RMS -0.0 dB, peak -0.0 dB, spectral 96.6 dB](/accuracy/circuits/canonical/bjt_diff_pair/linear.png)

*canonical/bjt_diff_pair/linear — RMS -0.0 dB, peak -0.0 dB, spectral 96.6 dB*

![canonical/bjt_diff_pair/soft_clipping — RMS -0.0 dB, peak -0.0 dB, spectral 134.3 dB](/accuracy/circuits/canonical/bjt_diff_pair/soft_clipping.png)

*canonical/bjt_diff_pair/soft_clipping — RMS -0.0 dB, peak -0.0 dB, spectral 134.3 dB*

![canonical/bjt_diff_pair/sweep — RMS 0.1 dB, peak -0.1 dB, spectral 184.8 dB](/accuracy/circuits/canonical/bjt_diff_pair/sweep.png)

*canonical/bjt_diff_pair/sweep — RMS 0.1 dB, peak -0.1 dB, spectral 184.8 dB*

![canonical/opamp_diode_clipper/linear — RMS 0.0 dB, peak 0.8 dB, spectral 215.4 dB](/accuracy/circuits/canonical/opamp_diode_clipper/linear.png)

*canonical/opamp_diode_clipper/linear — RMS 0.0 dB, peak 0.8 dB, spectral 215.4 dB*

![canonical/opamp_diode_clipper/clipping — RMS 0.1 dB, peak 1.2 dB, spectral 208.6 dB](/accuracy/circuits/canonical/opamp_diode_clipper/clipping.png)

*canonical/opamp_diode_clipper/clipping — RMS 0.1 dB, peak 1.2 dB, spectral 208.6 dB*

![canonical/opamp_diode_clipper/sweep — RMS 0.1 dB, peak 1.2 dB, spectral 119.3 dB](/accuracy/circuits/canonical/opamp_diode_clipper/sweep.png)

*canonical/opamp_diode_clipper/sweep — RMS 0.1 dB, peak 1.2 dB, spectral 119.3 dB*

![canonical/rc_highpass/impulse — RMS -247.2 dB, peak -247.6 dB, spectral 0.0 dB](/accuracy/circuits/canonical/rc_highpass/impulse.png)

*canonical/rc_highpass/impulse — RMS -247.2 dB, peak -247.6 dB, spectral 0.0 dB*

![canonical/rc_highpass/sine — RMS -283.5 dB, peak -282.6 dB, spectral 0.0 dB](/accuracy/circuits/canonical/rc_highpass/sine.png)

*canonical/rc_highpass/sine — RMS -283.5 dB, peak -282.6 dB, spectral 0.0 dB*

![canonical/rc_highpass/sweep — RMS -279.7 dB, peak -275.7 dB, spectral 0.0 dB](/accuracy/circuits/canonical/rc_highpass/sweep.png)

*canonical/rc_highpass/sweep — RMS -279.7 dB, peak -275.7 dB, spectral 0.0 dB*

![canonical/rc_lowpass/impulse — RMS -200.0 dB, peak -200.0 dB, spectral 0.0 dB](/accuracy/circuits/canonical/rc_lowpass/impulse.png)

*canonical/rc_lowpass/impulse — RMS -200.0 dB, peak -200.0 dB, spectral 0.0 dB*

![canonical/rc_lowpass/sine — RMS -307.7 dB, peak -297.6 dB, spectral 0.0 dB](/accuracy/circuits/canonical/rc_lowpass/sine.png)

*canonical/rc_lowpass/sine — RMS -307.7 dB, peak -297.6 dB, spectral 0.0 dB*

![canonical/rc_lowpass/sweep — RMS -308.3 dB, peak -298.3 dB, spectral 0.0 dB](/accuracy/circuits/canonical/rc_lowpass/sweep.png)

*canonical/rc_lowpass/sweep — RMS -308.3 dB, peak -298.3 dB, spectral 0.0 dB*

![canonical/series_rlc/impulse — RMS -205.6 dB, peak -206.0 dB, spectral 0.0 dB](/accuracy/circuits/canonical/series_rlc/impulse.png)

*canonical/series_rlc/impulse — RMS -205.6 dB, peak -206.0 dB, spectral 0.0 dB*

![canonical/series_rlc/sine — RMS -287.7 dB, peak -285.0 dB, spectral 0.0 dB](/accuracy/circuits/canonical/series_rlc/sine.png)

*canonical/series_rlc/sine — RMS -287.7 dB, peak -285.0 dB, spectral 0.0 dB*

![canonical/series_rlc/sweep — RMS -230.7 dB, peak -225.9 dB, spectral 0.0 dB](/accuracy/circuits/canonical/series_rlc/sweep.png)

*canonical/series_rlc/sweep — RMS -230.7 dB, peak -225.9 dB, spectral 0.0 dB*

![canonical/single_diode/mild_clipping — RMS -25.6 dB, peak -26.9 dB, spectral 218.0 dB](/accuracy/circuits/canonical/single_diode/mild_clipping.png)

*canonical/single_diode/mild_clipping — RMS -25.6 dB, peak -26.9 dB, spectral 218.0 dB*

![canonical/single_diode/hard_clipping — RMS -40.1 dB, peak -40.7 dB, spectral 172.5 dB](/accuracy/circuits/canonical/single_diode/hard_clipping.png)

*canonical/single_diode/hard_clipping — RMS -40.1 dB, peak -40.7 dB, spectral 172.5 dB*

![canonical/single_diode/sweep — RMS -5.2 dB, peak 0.2 dB, spectral 69.3 dB](/accuracy/circuits/canonical/single_diode/sweep.png)

*canonical/single_diode/sweep — RMS -5.2 dB, peak 0.2 dB, spectral 69.3 dB*

![canonical/transformer_load/sine — RMS -90.7 dB, peak -90.1 dB, spectral 0.0 dB](/accuracy/circuits/canonical/transformer_load/sine.png)

*canonical/transformer_load/sine — RMS -90.7 dB, peak -90.1 dB, spectral 0.0 dB*

![canonical/transformer_load/sweep — RMS -60.3 dB, peak -49.8 dB, spectral 0.1 dB](/accuracy/circuits/canonical/transformer_load/sweep.png)

*canonical/transformer_load/sweep — RMS -60.3 dB, peak -49.8 dB, spectral 0.1 dB*

![canonical/twin_t_notch/impulse — RMS -236.0 dB, peak -236.2 dB, spectral 0.0 dB](/accuracy/circuits/canonical/twin_t_notch/impulse.png)

*canonical/twin_t_notch/impulse — RMS -236.0 dB, peak -236.2 dB, spectral 0.0 dB*

![canonical/twin_t_notch/sine — RMS -242.4 dB, peak -237.0 dB, spectral 0.0 dB](/accuracy/circuits/canonical/twin_t_notch/sine.png)

*canonical/twin_t_notch/sine — RMS -242.4 dB, peak -237.0 dB, spectral 0.0 dB*

![canonical/twin_t_notch/sweep — RMS -269.1 dB, peak -262.6 dB, spectral 0.0 dB](/accuracy/circuits/canonical/twin_t_notch/sweep.png)

*canonical/twin_t_notch/sweep — RMS -269.1 dB, peak -262.6 dB, spectral 0.0 dB*

### compressor

| Circuit / signal | Pass | RMS (dB) | Peak (dB) | Spec (dB) | THD (dB) |
|------------------|------|----------|-----------|-----------|----------|
| fet_leveler_level_sweep / level_sweep | **FAIL** | 0.0 | 0.0 | 245.5 | 90.8 |
| fet_leveler_tone_burst / tone_burst | **FAIL** | 0.0 | 0.1 | 153.4 | 47.4 |

![compressor/fet_leveler_level_sweep/level_sweep — RMS 0.0 dB, peak 0.0 dB, spectral 245.5 dB](/accuracy/circuits/compressor/fet_leveler_level_sweep/level_sweep.png)

*compressor/fet_leveler_level_sweep/level_sweep — RMS 0.0 dB, peak 0.0 dB, spectral 245.5 dB*

![compressor/fet_leveler_tone_burst/tone_burst — RMS 0.0 dB, peak 0.1 dB, spectral 153.4 dB](/accuracy/circuits/compressor/fet_leveler_tone_burst/tone_burst.png)

*compressor/fet_leveler_tone_burst/tone_burst — RMS 0.0 dB, peak 0.1 dB, spectral 153.4 dB*

### eq

| Circuit / signal | Pass | RMS (dB) | Peak (dB) | Spec (dB) | THD (dB) |
|------------------|------|----------|-----------|-----------|----------|
| pultec_passive_flat / sine_1k | PASS | -76.9 | -76.8 | 0.0 | 0.1 |
| pultec_passive_flat / sweep | PASS | -48.3 | -46.0 | 1.1 | — |
| pultec_passive_hf_atten / sine_10k | PASS | -46.5 | -46.4 | 0.0 | 105.3 |
| pultec_passive_hf_atten / sweep | PASS | -49.9 | -48.2 | 0.5 | — |
| pultec_passive_hf_boost / sine_3k1 | PASS | -68.3 | -68.0 | 0.0 | 80.1 |
| pultec_passive_hf_boost / sweep | PASS | -65.4 | -55.3 | 0.1 | — |
| pultec_passive_lf_boost / sine_60 | PASS | -103.7 | -102.8 | 0.0 | 0.8 |
| pultec_passive_lf_boost / sweep | PASS | -48.3 | -46.0 | 1.1 | — |
| pultec_passive_lf_trick / sine_60 | PASS | -103.7 | -102.0 | 0.0 | 0.7 |
| pultec_passive_lf_trick / sweep | PASS | -47.8 | -46.1 | 2.2 | — |

![eq/pultec_passive_flat/sine_1k — RMS -76.9 dB, peak -76.8 dB, spectral 0.0 dB](/accuracy/circuits/eq/pultec_passive_flat/sine_1k.png)

*eq/pultec_passive_flat/sine_1k — RMS -76.9 dB, peak -76.8 dB, spectral 0.0 dB*

![eq/pultec_passive_flat/sweep — RMS -48.3 dB, peak -46.0 dB, spectral 1.1 dB](/accuracy/circuits/eq/pultec_passive_flat/sweep.png)

*eq/pultec_passive_flat/sweep — RMS -48.3 dB, peak -46.0 dB, spectral 1.1 dB*

![eq/pultec_passive_hf_atten/sine_10k — RMS -46.5 dB, peak -46.4 dB, spectral 0.0 dB](/accuracy/circuits/eq/pultec_passive_hf_atten/sine_10k.png)

*eq/pultec_passive_hf_atten/sine_10k — RMS -46.5 dB, peak -46.4 dB, spectral 0.0 dB*

![eq/pultec_passive_hf_atten/sweep — RMS -49.9 dB, peak -48.2 dB, spectral 0.5 dB](/accuracy/circuits/eq/pultec_passive_hf_atten/sweep.png)

*eq/pultec_passive_hf_atten/sweep — RMS -49.9 dB, peak -48.2 dB, spectral 0.5 dB*

![eq/pultec_passive_hf_boost/sine_3k1 — RMS -68.3 dB, peak -68.0 dB, spectral 0.0 dB](/accuracy/circuits/eq/pultec_passive_hf_boost/sine_3k1.png)

*eq/pultec_passive_hf_boost/sine_3k1 — RMS -68.3 dB, peak -68.0 dB, spectral 0.0 dB*

![eq/pultec_passive_hf_boost/sweep — RMS -65.4 dB, peak -55.3 dB, spectral 0.1 dB](/accuracy/circuits/eq/pultec_passive_hf_boost/sweep.png)

*eq/pultec_passive_hf_boost/sweep — RMS -65.4 dB, peak -55.3 dB, spectral 0.1 dB*

![eq/pultec_passive_lf_boost/sine_60 — RMS -103.7 dB, peak -102.8 dB, spectral 0.0 dB](/accuracy/circuits/eq/pultec_passive_lf_boost/sine_60.png)

*eq/pultec_passive_lf_boost/sine_60 — RMS -103.7 dB, peak -102.8 dB, spectral 0.0 dB*

![eq/pultec_passive_lf_boost/sweep — RMS -48.3 dB, peak -46.0 dB, spectral 1.1 dB](/accuracy/circuits/eq/pultec_passive_lf_boost/sweep.png)

*eq/pultec_passive_lf_boost/sweep — RMS -48.3 dB, peak -46.0 dB, spectral 1.1 dB*

![eq/pultec_passive_lf_trick/sine_60 — RMS -103.7 dB, peak -102.0 dB, spectral 0.0 dB](/accuracy/circuits/eq/pultec_passive_lf_trick/sine_60.png)

*eq/pultec_passive_lf_trick/sine_60 — RMS -103.7 dB, peak -102.0 dB, spectral 0.0 dB*

![eq/pultec_passive_lf_trick/sweep — RMS -47.8 dB, peak -46.1 dB, spectral 2.2 dB](/accuracy/circuits/eq/pultec_passive_lf_trick/sweep.png)

*eq/pultec_passive_lf_trick/sweep — RMS -47.8 dB, peak -46.1 dB, spectral 2.2 dB*

### extraction

| Circuit / signal | Pass | RMS (dB) | Peak (dB) | Spec (dB) | THD (dB) |
|------------------|------|----------|-----------|-----------|----------|
| active_feedback_treble_shelf / sine | PASS | 5.5 | 5.5 | 0.9 | 69.6 |
| active_feedback_treble_shelf / sweep | PASS | 5.2 | 5.0 | 2.3 | — |
| mfb_highpass_controlled / sine | PASS | 0.9 | 0.9 | 3.7 | 52.5 |
| mfb_highpass_controlled / sweep | **FAIL** | -0.0 | -0.0 | 40.1 | — |
| output_wiper_divider / sine | PASS | -100.0 | -100.0 | 0.0 | 0.0 |
| passive_loaded_rc_lowpass / sine | PASS | -83.3 | -83.2 | 0.0 | 0.2 |
| passive_loaded_rc_lowpass / sweep | PASS | -77.2 | -72.2 | 0.0 | — |
| sallen_key_highpass / sine | **FAIL** | 13.4 | 13.2 | 11.3 | 83.9 |
| sallen_key_highpass / sweep | **FAIL** | 13.4 | 13.1 | 44.6 | — |
| sallen_key_lowpass / sine | PASS | 5.3 | 5.2 | 1.5 | 84.7 |
| sallen_key_lowpass / sweep | PASS | 5.0 | 4.6 | 2.9 | — |

![extraction/active_feedback_treble_shelf/sine — RMS 5.5 dB, peak 5.5 dB, spectral 0.9 dB](/accuracy/circuits/extraction/active_feedback_treble_shelf/sine.png)

*extraction/active_feedback_treble_shelf/sine — RMS 5.5 dB, peak 5.5 dB, spectral 0.9 dB*

![extraction/active_feedback_treble_shelf/sweep — RMS 5.2 dB, peak 5.0 dB, spectral 2.3 dB](/accuracy/circuits/extraction/active_feedback_treble_shelf/sweep.png)

*extraction/active_feedback_treble_shelf/sweep — RMS 5.2 dB, peak 5.0 dB, spectral 2.3 dB*

![extraction/mfb_highpass_controlled/sine — RMS 0.9 dB, peak 0.9 dB, spectral 3.7 dB](/accuracy/circuits/extraction/mfb_highpass_controlled/sine.png)

*extraction/mfb_highpass_controlled/sine — RMS 0.9 dB, peak 0.9 dB, spectral 3.7 dB*

![extraction/mfb_highpass_controlled/sweep — RMS -0.0 dB, peak -0.0 dB, spectral 40.1 dB](/accuracy/circuits/extraction/mfb_highpass_controlled/sweep.png)

*extraction/mfb_highpass_controlled/sweep — RMS -0.0 dB, peak -0.0 dB, spectral 40.1 dB*

![extraction/output_wiper_divider/sine — RMS -100.0 dB, peak -100.0 dB, spectral 0.0 dB](/accuracy/circuits/extraction/output_wiper_divider/sine.png)

*extraction/output_wiper_divider/sine — RMS -100.0 dB, peak -100.0 dB, spectral 0.0 dB*

![extraction/passive_loaded_rc_lowpass/sine — RMS -83.3 dB, peak -83.2 dB, spectral 0.0 dB](/accuracy/circuits/extraction/passive_loaded_rc_lowpass/sine.png)

*extraction/passive_loaded_rc_lowpass/sine — RMS -83.3 dB, peak -83.2 dB, spectral 0.0 dB*

![extraction/passive_loaded_rc_lowpass/sweep — RMS -77.2 dB, peak -72.2 dB, spectral 0.0 dB](/accuracy/circuits/extraction/passive_loaded_rc_lowpass/sweep.png)

*extraction/passive_loaded_rc_lowpass/sweep — RMS -77.2 dB, peak -72.2 dB, spectral 0.0 dB*

![extraction/sallen_key_highpass/sine — RMS 13.4 dB, peak 13.2 dB, spectral 11.3 dB](/accuracy/circuits/extraction/sallen_key_highpass/sine.png)

*extraction/sallen_key_highpass/sine — RMS 13.4 dB, peak 13.2 dB, spectral 11.3 dB*

![extraction/sallen_key_highpass/sweep — RMS 13.4 dB, peak 13.1 dB, spectral 44.6 dB](/accuracy/circuits/extraction/sallen_key_highpass/sweep.png)

*extraction/sallen_key_highpass/sweep — RMS 13.4 dB, peak 13.1 dB, spectral 44.6 dB*

![extraction/sallen_key_lowpass/sine — RMS 5.3 dB, peak 5.2 dB, spectral 1.5 dB](/accuracy/circuits/extraction/sallen_key_lowpass/sine.png)

*extraction/sallen_key_lowpass/sine — RMS 5.3 dB, peak 5.2 dB, spectral 1.5 dB*

![extraction/sallen_key_lowpass/sweep — RMS 5.0 dB, peak 4.6 dB, spectral 2.9 dB](/accuracy/circuits/extraction/sallen_key_lowpass/sweep.png)

*extraction/sallen_key_lowpass/sweep — RMS 5.0 dB, peak 4.6 dB, spectral 2.9 dB*

### linear

| Circuit / signal | Pass | RMS (dB) | Peak (dB) | Spec (dB) | THD (dB) |
|------------------|------|----------|-----------|-----------|----------|
| rc_highpass / sine | PASS | -305.5 | -297.4 | 0.0 | 0.0 |
| rc_highpass / sweep | PASS | -301.8 | -292.2 | 0.0 | — |
| rc_lowpass / impulse | PASS | -200.0 | -200.0 | 0.0 | — |
| rc_lowpass / sine | PASS | -307.7 | -297.6 | 0.0 | 0.0 |
| rc_lowpass / sweep | PASS | -308.3 | -298.3 | 0.0 | — |
| resistor_divider / sine | PASS | -317.5 | -319.1 | 0.0 | 0.0 |
| rl_lowpass / sine | PASS | -307.5 | -297.6 | 0.0 | 0.0 |

![linear/rc_highpass/sine — RMS -305.5 dB, peak -297.4 dB, spectral 0.0 dB](/accuracy/circuits/linear/rc_highpass/sine.png)

*linear/rc_highpass/sine — RMS -305.5 dB, peak -297.4 dB, spectral 0.0 dB*

![linear/rc_highpass/sweep — RMS -301.8 dB, peak -292.2 dB, spectral 0.0 dB](/accuracy/circuits/linear/rc_highpass/sweep.png)

*linear/rc_highpass/sweep — RMS -301.8 dB, peak -292.2 dB, spectral 0.0 dB*

![linear/rc_lowpass/impulse — RMS -200.0 dB, peak -200.0 dB, spectral 0.0 dB](/accuracy/circuits/linear/rc_lowpass/impulse.png)

*linear/rc_lowpass/impulse — RMS -200.0 dB, peak -200.0 dB, spectral 0.0 dB*

![linear/rc_lowpass/sine — RMS -307.7 dB, peak -297.6 dB, spectral 0.0 dB](/accuracy/circuits/linear/rc_lowpass/sine.png)

*linear/rc_lowpass/sine — RMS -307.7 dB, peak -297.6 dB, spectral 0.0 dB*

![linear/rc_lowpass/sweep — RMS -308.3 dB, peak -298.3 dB, spectral 0.0 dB](/accuracy/circuits/linear/rc_lowpass/sweep.png)

*linear/rc_lowpass/sweep — RMS -308.3 dB, peak -298.3 dB, spectral 0.0 dB*

![linear/resistor_divider/sine — RMS -317.5 dB, peak -319.1 dB, spectral 0.0 dB](/accuracy/circuits/linear/resistor_divider/sine.png)

*linear/resistor_divider/sine — RMS -317.5 dB, peak -319.1 dB, spectral 0.0 dB*

![linear/rl_lowpass/sine — RMS -307.5 dB, peak -297.6 dB, spectral 0.0 dB](/accuracy/circuits/linear/rl_lowpass/sine.png)

*linear/rl_lowpass/sine — RMS -307.5 dB, peak -297.6 dB, spectral 0.0 dB*

### nonlinear

| Circuit / signal | Pass | RMS (dB) | Peak (dB) | Spec (dB) | THD (dB) |
|------------------|------|----------|-----------|-----------|----------|
| common_cathode_12ax7 / clean | PASS | -0.1 | -0.8 | 37.5 | 0.7 |
| common_cathode_12ax7 / driven | PASS | -2.8 | -4.1 | 131.5 | 1.6 |
| diode_clipper / low_level | PASS | -21.7 | -21.7 | 172.4 | 21.3 |
| diode_clipper / clipping | PASS | -2.6 | 5.4 | 143.8 | 0.0 |
| diode_clipper / sweep | PASS | -0.9 | 7.0 | 41.6 | — |
| diode_no_cap / low_level | PASS | -23.5 | -20.3 | 172.2 | 21.0 |
| diode_no_cap / clipping | PASS | -11.9 | -8.7 | 112.6 | 0.2 |
| single_diode / sine | PASS | -10.4 | -8.3 | 170.0 | 4.0 |
| zener_clipper / low_level | PASS | -19.8 | -17.6 | 174.8 | 25.1 |
| zener_clipper / clipping | PASS | -7.7 | 2.5 | 142.2 | 0.2 |

![nonlinear/common_cathode_12ax7/clean — RMS -0.1 dB, peak -0.8 dB, spectral 37.5 dB](/accuracy/circuits/nonlinear/common_cathode_12ax7/clean.png)

*nonlinear/common_cathode_12ax7/clean — RMS -0.1 dB, peak -0.8 dB, spectral 37.5 dB*

![nonlinear/common_cathode_12ax7/driven — RMS -2.8 dB, peak -4.1 dB, spectral 131.5 dB](/accuracy/circuits/nonlinear/common_cathode_12ax7/driven.png)

*nonlinear/common_cathode_12ax7/driven — RMS -2.8 dB, peak -4.1 dB, spectral 131.5 dB*

![nonlinear/diode_clipper/low_level — RMS -21.7 dB, peak -21.7 dB, spectral 172.4 dB](/accuracy/circuits/nonlinear/diode_clipper/low_level.png)

*nonlinear/diode_clipper/low_level — RMS -21.7 dB, peak -21.7 dB, spectral 172.4 dB*

![nonlinear/diode_clipper/clipping — RMS -2.6 dB, peak 5.4 dB, spectral 143.8 dB](/accuracy/circuits/nonlinear/diode_clipper/clipping.png)

*nonlinear/diode_clipper/clipping — RMS -2.6 dB, peak 5.4 dB, spectral 143.8 dB*

![nonlinear/diode_clipper/sweep — RMS -0.9 dB, peak 7.0 dB, spectral 41.6 dB](/accuracy/circuits/nonlinear/diode_clipper/sweep.png)

*nonlinear/diode_clipper/sweep — RMS -0.9 dB, peak 7.0 dB, spectral 41.6 dB*

![nonlinear/diode_no_cap/low_level — RMS -23.5 dB, peak -20.3 dB, spectral 172.2 dB](/accuracy/circuits/nonlinear/diode_no_cap/low_level.png)

*nonlinear/diode_no_cap/low_level — RMS -23.5 dB, peak -20.3 dB, spectral 172.2 dB*

![nonlinear/diode_no_cap/clipping — RMS -11.9 dB, peak -8.7 dB, spectral 112.6 dB](/accuracy/circuits/nonlinear/diode_no_cap/clipping.png)

*nonlinear/diode_no_cap/clipping — RMS -11.9 dB, peak -8.7 dB, spectral 112.6 dB*

![nonlinear/single_diode/sine — RMS -10.4 dB, peak -8.3 dB, spectral 170.0 dB](/accuracy/circuits/nonlinear/single_diode/sine.png)

*nonlinear/single_diode/sine — RMS -10.4 dB, peak -8.3 dB, spectral 170.0 dB*

![nonlinear/zener_clipper/low_level — RMS -19.8 dB, peak -17.6 dB, spectral 174.8 dB](/accuracy/circuits/nonlinear/zener_clipper/low_level.png)

*nonlinear/zener_clipper/low_level — RMS -19.8 dB, peak -17.6 dB, spectral 174.8 dB*

![nonlinear/zener_clipper/clipping — RMS -7.7 dB, peak 2.5 dB, spectral 142.2 dB](/accuracy/circuits/nonlinear/zener_clipper/clipping.png)

*nonlinear/zener_clipper/clipping — RMS -7.7 dB, peak 2.5 dB, spectral 142.2 dB*

### opamp

| Circuit / signal | Pass | RMS (dB) | Peak (dB) | Spec (dB) | THD (dB) |
|------------------|------|----------|-----------|-----------|----------|
| difference_amp / sine | PASS | -31.6 | -16.7 | 202.7 | 135.4 |
| integrator / sine | PASS | 1.9 | 2.2 | 176.8 | 83.5 |
| inverting_gain10 / sine | PASS | -31.6 | -16.9 | 202.4 | 135.5 |
| noninverting_gain10 / sine | PASS | -31.6 | -16.9 | 202.4 | 135.5 |
| summing_inverting / sine | PASS | -31.6 | -16.7 | 202.7 | 135.5 |
| unity_buffer / sine | PASS | -49.8 | -16.7 | 202.7 | 0.7 |

![opamp/difference_amp/sine — RMS -31.6 dB, peak -16.7 dB, spectral 202.7 dB](/accuracy/circuits/opamp/difference_amp/sine.png)

*opamp/difference_amp/sine — RMS -31.6 dB, peak -16.7 dB, spectral 202.7 dB*

![opamp/integrator/sine — RMS 1.9 dB, peak 2.2 dB, spectral 176.8 dB](/accuracy/circuits/opamp/integrator/sine.png)

*opamp/integrator/sine — RMS 1.9 dB, peak 2.2 dB, spectral 176.8 dB*

![opamp/inverting_gain10/sine — RMS -31.6 dB, peak -16.9 dB, spectral 202.4 dB](/accuracy/circuits/opamp/inverting_gain10/sine.png)

*opamp/inverting_gain10/sine — RMS -31.6 dB, peak -16.9 dB, spectral 202.4 dB*

![opamp/noninverting_gain10/sine — RMS -31.6 dB, peak -16.9 dB, spectral 202.4 dB](/accuracy/circuits/opamp/noninverting_gain10/sine.png)

*opamp/noninverting_gain10/sine — RMS -31.6 dB, peak -16.9 dB, spectral 202.4 dB*

![opamp/summing_inverting/sine — RMS -31.6 dB, peak -16.7 dB, spectral 202.7 dB](/accuracy/circuits/opamp/summing_inverting/sine.png)

*opamp/summing_inverting/sine — RMS -31.6 dB, peak -16.7 dB, spectral 202.7 dB*

![opamp/unity_buffer/sine — RMS -49.8 dB, peak -16.7 dB, spectral 202.7 dB](/accuracy/circuits/opamp/unity_buffer/sine.png)

*opamp/unity_buffer/sine — RMS -49.8 dB, peak -16.7 dB, spectral 202.7 dB*

### pedals

| Circuit / signal | Pass | RMS (dB) | Peak (dB) | Spec (dB) | THD (dB) |
|------------------|------|----------|-----------|-----------|----------|
| bigmuff_stage / clean | **FAIL** | 70.2 | 50.7 | 116.4 | 94.5 |
| bigmuff_stage / clipping | **FAIL** | 90.2 | 70.7 | 116.4 | 69.0 |
| fuzz_core / clean | **FAIL** | 7.7 | 1.8 | 145.5 | 110.7 |
| fuzz_core / saturated | **FAIL** | 26.8 | 12.4 | 145.5 | 58.4 |
| rat_clipper / clean | PASS | 5.3 | 6.8 | 182.4 | 14.3 |
| rat_clipper / clipping | PASS | 3.4 | 6.1 | 162.8 | 3.0 |
| ts808_clipper / clean | PASS | 8.1 | 8.7 | 187.9 | 2.1 |
| ts808_clipper / clipping | PASS | 7.6 | 8.4 | 167.7 | 0.9 |

![pedals/bigmuff_stage/clean — RMS 70.2 dB, peak 50.7 dB, spectral 116.4 dB](/accuracy/circuits/pedals/bigmuff_stage/clean.png)

*pedals/bigmuff_stage/clean — RMS 70.2 dB, peak 50.7 dB, spectral 116.4 dB*

![pedals/bigmuff_stage/clipping — RMS 90.2 dB, peak 70.7 dB, spectral 116.4 dB](/accuracy/circuits/pedals/bigmuff_stage/clipping.png)

*pedals/bigmuff_stage/clipping — RMS 90.2 dB, peak 70.7 dB, spectral 116.4 dB*

![pedals/fuzz_core/clean — RMS 7.7 dB, peak 1.8 dB, spectral 145.5 dB](/accuracy/circuits/pedals/fuzz_core/clean.png)

*pedals/fuzz_core/clean — RMS 7.7 dB, peak 1.8 dB, spectral 145.5 dB*

![pedals/fuzz_core/saturated — RMS 26.8 dB, peak 12.4 dB, spectral 145.5 dB](/accuracy/circuits/pedals/fuzz_core/saturated.png)

*pedals/fuzz_core/saturated — RMS 26.8 dB, peak 12.4 dB, spectral 145.5 dB*

![pedals/rat_clipper/clean — RMS 5.3 dB, peak 6.8 dB, spectral 182.4 dB](/accuracy/circuits/pedals/rat_clipper/clean.png)

*pedals/rat_clipper/clean — RMS 5.3 dB, peak 6.8 dB, spectral 182.4 dB*

![pedals/rat_clipper/clipping — RMS 3.4 dB, peak 6.1 dB, spectral 162.8 dB](/accuracy/circuits/pedals/rat_clipper/clipping.png)

*pedals/rat_clipper/clipping — RMS 3.4 dB, peak 6.1 dB, spectral 162.8 dB*

![pedals/ts808_clipper/clean — RMS 8.1 dB, peak 8.7 dB, spectral 187.9 dB](/accuracy/circuits/pedals/ts808_clipper/clean.png)

*pedals/ts808_clipper/clean — RMS 8.1 dB, peak 8.7 dB, spectral 187.9 dB*

![pedals/ts808_clipper/clipping — RMS 7.6 dB, peak 8.4 dB, spectral 167.7 dB](/accuracy/circuits/pedals/ts808_clipper/clipping.png)

*pedals/ts808_clipper/clipping — RMS 7.6 dB, peak 8.4 dB, spectral 167.7 dB*

### reactive

| Circuit / signal | Pass | RMS (dB) | Peak (dB) | Spec (dB) | THD (dB) |
|------------------|------|----------|-----------|-----------|----------|
| delay_simple / impulse | PASS | 0.0 | 0.0 | 496.3 | — |
| lc_resonant / resonant | PASS | -44.9 | -23.4 | 162.1 | 48.1 |
| lc_resonant / sweep | PASS | -5.3 | 5.4 | 223.1 | — |
| transformer_stepdown / sine | PASS | -98.6 | -97.9 | 0.0 | 35.1 |
| transformer_stepdown / sweep | PASS | -59.1 | -46.0 | 0.1 | — |
| transformer_stepup / sine | PASS | -90.7 | -90.2 | 0.0 | 56.1 |

![reactive/delay_simple/impulse — RMS 0.0 dB, peak 0.0 dB, spectral 496.3 dB](/accuracy/circuits/reactive/delay_simple/impulse.png)

*reactive/delay_simple/impulse — RMS 0.0 dB, peak 0.0 dB, spectral 496.3 dB*

![reactive/lc_resonant/resonant — RMS -44.9 dB, peak -23.4 dB, spectral 162.1 dB](/accuracy/circuits/reactive/lc_resonant/resonant.png)

*reactive/lc_resonant/resonant — RMS -44.9 dB, peak -23.4 dB, spectral 162.1 dB*

![reactive/lc_resonant/sweep — RMS -5.3 dB, peak 5.4 dB, spectral 223.1 dB](/accuracy/circuits/reactive/lc_resonant/sweep.png)

*reactive/lc_resonant/sweep — RMS -5.3 dB, peak 5.4 dB, spectral 223.1 dB*

![reactive/transformer_stepdown/sine — RMS -98.6 dB, peak -97.9 dB, spectral 0.0 dB](/accuracy/circuits/reactive/transformer_stepdown/sine.png)

*reactive/transformer_stepdown/sine — RMS -98.6 dB, peak -97.9 dB, spectral 0.0 dB*

![reactive/transformer_stepdown/sweep — RMS -59.1 dB, peak -46.0 dB, spectral 0.1 dB](/accuracy/circuits/reactive/transformer_stepdown/sweep.png)

*reactive/transformer_stepdown/sweep — RMS -59.1 dB, peak -46.0 dB, spectral 0.1 dB*

![reactive/transformer_stepup/sine — RMS -90.7 dB, peak -90.2 dB, spectral 0.0 dB](/accuracy/circuits/reactive/transformer_stepup/sine.png)

*reactive/transformer_stepup/sine — RMS -90.7 dB, peak -90.2 dB, spectral 0.0 dB*

### stress

| Circuit / signal | Pass | RMS (dB) | Peak (dB) | Spec (dB) | THD (dB) |
|------------------|------|----------|-----------|-----------|----------|
| dc_stability / silence | PASS | -200.0 | -200.0 | 0.0 | — |

![stress/dc_stability/silence — RMS -200.0 dB, peak -200.0 dB, spectral 0.0 dB](/accuracy/circuits/stress/dc_stability/silence.png)

*stress/dc_stability/silence — RMS -200.0 dB, peak -200.0 dB, spectral 0.0 dB*

### tape

| Circuit / signal | Pass | RMS (dB) | Peak (dB) | Spec (dB) | THD (dB) |
|------------------|------|----------|-----------|-----------|----------|
| tape_head_saturation / clean | PASS | -72.2 | -72.9 | 0.2 | 0.0 |
| tape_head_saturation / saturated | PASS | -69.6 | -69.8 | 0.0 | 0.0 |

![tape/tape_head_saturation/clean — RMS -72.2 dB, peak -72.9 dB, spectral 0.2 dB](/accuracy/circuits/tape/tape_head_saturation/clean.png)

*tape/tape_head_saturation/clean — RMS -72.2 dB, peak -72.9 dB, spectral 0.2 dB*

![tape/tape_head_saturation/saturated — RMS -69.6 dB, peak -69.8 dB, spectral 0.0 dB](/accuracy/circuits/tape/tape_head_saturation/saturated.png)

*tape/tape_head_saturation/saturated — RMS -69.6 dB, peak -69.8 dB, spectral 0.0 dB*

---

Generated by `tools/dashboard/generate_dashboard.py` from committed WDF + ngspice goldens (`pedalkernel-validate compare-goldens`). Snapshot commit `0028e6cee`, run `2026-06-15T00:00:00Z`. Machine-readable data: [`data/accuracy.json`](https://github.com/ajmwagar/pedalkernel) and `data/accuracy-history.json` in the repo.
